#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Exclusive map->odom publisher for selected-wall or full-map matching."""
import json
import math
import threading
import time
import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String
from wheel_distance import PulseBuffer, wheel_prediction, distance_prior, wrap
from wall_features import (DEFAULTS, compose, inverse, fingerprint, validate_targets,
                           match, scan_points)
from map_wall_extraction import map_geometry


class WallLocalizer(object):
    def __init__(self):
        reference=rospy.get_param('~parameter_source','')
        def setting(name,default):
            inherited=rospy.get_param(reference+'/'+name[1:],default) if reference else default
            return rospy.get_param(name,inherited)
        self.lock=threading.RLock()
        self.frame=setting('~map_frame','map')
        self.base=setting('~base_frame','base_footprint')
        self.odom=setting('~odom_frame','odom')
        self.coast=float(setting('~coast_timeout',2.))
        self.scan_timeout=float(setting('~scan_timeout',.5))
        self.switch_delay=float(setting('~state_confirm_time',.3))
        self.options={k:setting('~'+k,v) for k,v in DEFAULTS.items()}
        for k,v in list(self.options.items())+[('coast_timeout',self.coast),('scan_timeout',self.scan_timeout),('state_confirm_time',self.switch_delay)]:
            if not np.isfinite(v) or v<=0: raise ValueError('Invalid positive parameter '+k)
        self.wheel_enabled=setting('~wheel_fusion',False)
        self.target_mode=setting('~target_mode','selected_walls')
        if self.target_mode not in ('selected_walls','full_map'):
            raise ValueError('Unknown target_mode: '+str(self.target_mode))
        if self.target_mode=='full_map' and self.wheel_enabled:
            raise ValueError('full_map uses scan odometry only; disable wheel_fusion')
        self.full_targets=[]
        self.broadcast_tf=setting('~broadcast_tf',True)
        self.base_offset=float(setting('~rear_offset',.31))
        self.wheel_sigma=float(setting('~wheel_sigma_floor',.005))
        self.wheel_relative_sigma=float(setting('~wheel_relative_sigma',.05))
        if self.wheel_sigma<=0 or self.wheel_relative_sigma<0 or not all(np.isfinite(v) for v in (self.base_offset,self.wheel_sigma,self.wheel_relative_sigma)):
            raise ValueError('Invalid wheel uncertainty configuration')
        self.pulse_lock=threading.RLock()
        self.pulses=PulseBuffer(float(setting('~ticks_per_meter',10000.)))
        self.wheel_previous=None;self.pulse_error=None
        if self.wheel_enabled: rospy.Subscriber('/wheel/pulses',JointState,self.on_pulses,queue_size=200)
        self.listener=tf.TransformListener();self.broadcaster=tf.TransformBroadcaster()
        self.status=rospy.Publisher('/wall_localization/status',String,queue_size=1,latch=True)
        self.pose_pub=rospy.Publisher('/wall_localization/pose',PoseWithCovarianceStamped,queue_size=1)
        self.map_id=None;self.groups=dict(inside=[],outside=[]);self.definition=None
        self.region_definition=None;self.polygon=[];self.correction=None;self.last_good=None;self.last_scan=None
        self.group=None;self.candidate=None;self.candidate_since=None
        self.last_clock=None
        rospy.Subscriber('/map',OccupancyGrid,self.on_map,queue_size=1)
        if self.target_mode=='selected_walls':
            rospy.Subscriber('/wall_features/definition',String,self.on_definition,queue_size=1)
            rospy.Subscriber('/external_region/definition',String,self.on_polygon,queue_size=1)
        rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,self.on_initial,queue_size=1)
        rospy.Subscriber('scan',LaserScan,self.on_scan,queue_size=1)
        self.watchdog=rospy.Timer(rospy.Duration(.2),self.on_timer)
        self.report('WAIT_INITIAL_POSE')

    def on_pulses(self,msg):
        with self.pulse_lock:
            try:
                if msg.name!=['rear_center_encoder_ticks'] or len(msg.position)!=1:
                    raise ValueError('Invalid encoder message')
                self.pulses.add(msg.header.stamp.to_sec(),msg.position[0],msg.header.frame_id)
                self.pulse_error=None
            except ValueError as exc: self.pulse_error=str(exc)

    def predict_with_wheel(self,odom,stamp,predicted):
        if not self.wheel_enabled: return predicted,None,'DISABLED'
        if self.wheel_previous is None: return predicted,None,'WARMUP'
        old_stamp,old_odom,old_pose,generation=self.wheel_previous
        deadline=time.time()+.06
        while True:
            with self.pulse_lock:
                try:
                    if generation!=self.pulses.generation: raise ValueError('Encoder reset')
                    if self.pulse_error: raise ValueError(self.pulse_error)
                    delta=self.pulses.delta(old_stamp,stamp)
                    break
                except ValueError as exc: reason=str(exc)
            if time.time()>=deadline: return predicted,None,'FALLBACK: '+reason
            time.sleep(.005)
        pose=wheel_prediction(old_pose,wrap(odom[2]-old_odom[2]),delta,self.base_offset)
        sigma=self.wheel_sigma+self.wheel_relative_sigma*abs(delta)
        return pose,distance_prior(old_pose,delta,self.base_offset,sigma),'FUSED'

    def report(self,state,**kwargs):
        kwargs.update(state=state,group=self.group,target_mode=self.target_mode,stamp=rospy.Time.now().to_sec())
        self.status.publish(json.dumps(kwargs,sort_keys=True))

    def read_tf(self,parent,child,stamp):
        p,q=self.listener.lookupTransform(parent,child,stamp)
        return (p[0],p[1],tf.transformations.euler_from_quaternion(q)[2])

    def apply_definition(self):
        self.groups=dict(inside=[],outside=[])
        if self.definition and self.definition.get('map_sha256')==self.map_id and self.definition.get('frame_id')==self.frame:
            self.groups=validate_targets(self.definition['groups'])

    def on_map(self,msg):
        with self.lock:
            key=fingerprint(msg)
            if key!=self.map_id:
                self.map_id=key;self.wheel_previous=None;self.correction=None;self.last_good=None;self.group=None
                self.full_targets=[]
                if self.target_mode=='full_map':
                    try:
                        if msg.header.frame_id.lstrip('/')!=self.frame:raise ValueError('Map frame mismatch')
                        cells,samples,res=map_geometry(msg)
                        if len(samples)<3:raise ValueError('Map has no usable wall surfaces')
                        # Same single full-map surface target used in the experiment.
                        self.full_targets=[dict(id=0,cells=cells.tolist(),samples=samples.tolist(),resolution=res)]
                    except ValueError as exc:
                        self.report('INVALID_MAP',reason=str(exc));return
                self.apply_region();self.apply_definition();self.report('WAIT_INITIAL_POSE',reason='Map changed')

    def on_definition(self,msg):
        with self.lock:
            try:
                data=json.loads(msg.data)
                if data.get('schema_version')!=3 or data.get('source')!='map_mask':
                    raise ValueError('Expected walls extracted from map boundaries')
                validate_targets(data['groups']);self.definition=data;self.apply_definition()
            except (ValueError,KeyError,TypeError) as exc:
                self.groups=dict(inside=[],outside=[]);self.report('INVALID_WALLS',reason=str(exc))

    def apply_region(self):
        self.polygon=[]
        data=self.region_definition
        if not data or data.get('map_sha256')!=self.map_id or data.get('frame_id')!=self.frame: return
        p=np.asarray(data.get('corners',[]),dtype=float)
        if p.shape!=(4,2) or not np.all(np.isfinite(p)): return
        crosses=[float(np.cross(p[(i+1)%4]-p[i],p[(i+2)%4]-p[(i+1)%4])) for i in range(4)]
        if all(c>.001 for c in crosses) or all(c<-.001 for c in crosses): self.polygon=p.tolist()

    def on_polygon(self,msg):
        with self.lock:
            try:
                self.region_definition=json.loads(msg.data);self.apply_region()
            except (ValueError,TypeError): self.polygon=[]

    def on_initial(self,msg):
        with self.lock:
            try:
                if not self.map_id: raise ValueError('Wait for map')
                if self.target_mode=='full_map' and not self.full_targets:raise ValueError('Wait for usable map surfaces')
                if msg.header.frame_id.lstrip('/')!=self.frame: raise ValueError('Initial pose frame mismatch')
                p=msg.pose.pose;q=p.orientation
                values=[p.position.x,p.position.y,q.x,q.y,q.z,q.w]
                if not np.all(np.isfinite(values)) or abs(np.linalg.norm(values[2:])-1.)>.01:
                    raise ValueError('Invalid initial pose')
                pose=(p.position.x,p.position.y,tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2])
                self.listener.waitForTransform(self.odom,self.base,msg.header.stamp,rospy.Duration(.2))
                odom=self.read_tf(self.odom,self.base,msg.header.stamp)
                self.correction=compose(pose,inverse(odom));self.last_good=rospy.Time.now()
                self.group=None;self.candidate=None;self.wheel_previous=None;self.report('INITIALIZED')
            except (tf.Exception,ValueError) as exc: self.report('INITIAL_POSE_REJECTED',reason=str(exc))

    def choose_group(self,pose,now):
        if not self.polygon:
            self.group=None;self.candidate=None;return None
        p=np.array(self.polygon);point=np.array(pose[:2])
        distances=[]
        for i in range(4):
            edge=p[(i+1)%4]-p[i]
            distances.append(float(np.cross(edge,point-p[i]))/np.linalg.norm(edge))
        inside=all(d>=-1e-9 for d in distances) or all(d<=1e-9 for d in distances)
        candidate='inside' if inside else 'outside'
        if candidate!=self.candidate:
            self.candidate=candidate;self.candidate_since=now
        if self.group is None or (now-self.candidate_since).to_sec()>=self.switch_delay:
            self.group=candidate
        return self.group if self.group==candidate else None

    def on_scan(self,msg):
        with self.lock:
            now=rospy.Time.now()
            if self.last_clock is not None and now<self.last_clock:
                self.correction=None;self.last_good=None;self.last_scan=None;self.wheel_previous=None;self.candidate=None;self.group=None
                self.report('WAIT_INITIAL_POSE',reason='Clock moved backwards')
            self.last_clock=now
            age=(now-msg.header.stamp).to_sec()
            if msg.header.stamp.to_sec()==0 or age<0 or age>self.scan_timeout:
                self.report('STALE_SCAN');return
            if self.last_scan is not None and msg.header.stamp<=self.last_scan:
                self.report('OUT_OF_ORDER_SCAN');return
            self.last_scan=msg.header.stamp
            if self.correction is None: self.report('WAIT_INITIAL_POSE');return
            try:
                # Scan odometry consumes this same scan; allow its TF to arrive.
                self.listener.waitForTransform(self.odom,self.base,msg.header.stamp,rospy.Duration(.2))
                odom=self.read_tf(self.odom,self.base,msg.header.stamp)
                # A fresh odom sample at the laser timestamp is mandatory.
                laser=self.read_tf(self.base,msg.header.frame_id,msg.header.stamp)
                predicted=compose(self.correction,odom)
                predicted,motion_prior,wheel_state=self.predict_with_wheel(odom,msg.header.stamp.to_sec(),predicted)
                group=self.choose_group(predicted,now) if self.target_mode=='selected_walls' else 'full_map'
                if self.target_mode=='full_map':self.group=group
                if group is None: fitted,detail=None,dict(reason='STATE_SWITCH' if self.polygon else 'NO_CORRIDOR_REGION')
                else:
                    points,normals=scan_points(msg,laser)
                    targets=self.full_targets if self.target_mode=='full_map' else self.groups[group]
                    fitted,detail=match(points,normals,predicted,targets,self.options,motion_prior=motion_prior)
                if fitted is not None:
                    self.correction=compose(fitted,inverse(odom));self.last_good=now
                    state=detail['reason']
                else:
                    state='ODOM_ONLY'
                    self.correction=compose(predicted,inverse(odom))
                detail['wheel_state']=wheel_state
                if self.last_good is None or (now-self.last_good).to_sec()>self.coast:
                    self.report('LOST',reason=detail['reason'],wheel_state=wheel_state);self.wheel_previous=None;return
                pose=compose(self.correction,odom)
                with self.pulse_lock:
                    self.wheel_previous=(msg.header.stamp.to_sec(),odom,pose,self.pulses.generation)
                if self.broadcast_tf: self.broadcaster.sendTransform((self.correction[0],self.correction[1],0.),
                    tf.transformations.quaternion_from_euler(0,0,self.correction[2]),
                    msg.header.stamp,self.odom,self.frame)
                output=PoseWithCovarianceStamped();output.header.frame_id=self.frame;output.header.stamp=msg.header.stamp
                output.pose.pose.position.x,output.pose.pose.position.y=pose[:2]
                q=tf.transformations.quaternion_from_euler(0,0,pose[2])
                output.pose.pose.orientation.x,output.pose.pose.orientation.y,output.pose.pose.orientation.z,output.pose.pose.orientation.w=q
                covariance=detail.pop('covariance',np.diag([1.,1.,1.]).tolist())
                for i,ii in enumerate((0,1,5)):
                    for j,jj in enumerate((0,1,5)): output.pose.covariance[ii*6+jj]=covariance[i][j]
                for i in (2,3,4): output.pose.covariance[i*6+i]=1e6
                self.pose_pub.publish(output)
                self.report(state,**detail)
            except (tf.Exception,ValueError, np.linalg.LinAlgError) as exc:
                self.report('INPUT_ERROR',reason=str(exc))

    def on_timer(self,event):
        with self.lock:
            if self.last_scan is None or (rospy.Time.now()-self.last_scan).to_sec()>self.scan_timeout:
                self.report('NO_FRESH_SCAN')


if __name__=='__main__':
    rospy.init_node('wall_localizer');WallLocalizer();rospy.spin()
