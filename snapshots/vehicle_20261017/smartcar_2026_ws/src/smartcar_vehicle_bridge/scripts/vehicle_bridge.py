#!/usr/bin/env python
from __future__ import division
import os,time,math,json,threading
from collections import deque
import rospy,rosgraph,tf
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32,Int64,String
from calibration import command,joints,validate,finite,interpolate
from feedback_monitor import PulseMonitor,PoseVelocity
from speed_observer import SpeedObserver
from standstill import Standstill

clock=getattr(time,'monotonic',lambda:os.times()[4])

class Bridge(object):
    def __init__(self):
        self.lock=threading.RLock()
        self.speeds=rospy.get_param('~speed_table');self.steering=rospy.get_param('~steering_table')
        self.reverse_steering=rospy.get_param("~reverse_steering_table",self.steering)
        self.direction=1
        self.steering_request=None
        self.command_speed=None;self.raw_velocity=None;self.steer_invalid=False;self.feedback_extrapolated=False
        self.feedback_raw_min=float(rospy.get_param('~steering_feedback_raw_min',1427.))
        self.feedback_raw_max=float(rospy.get_param('~steering_feedback_raw_max',2793.))
        if not (finite(self.feedback_raw_min) and finite(self.feedback_raw_max)
                and self.feedback_raw_min<self.feedback_raw_max
                and all(self.feedback_raw_min<=t[0][0]<t[-1][0]<=self.feedback_raw_max for t in (self.steering,self.reverse_steering))):
            raise ValueError('invalid physical steering feedback bounds')
        self.standstill=Standstill(window=float(rospy.get_param('~standstill_window',.35)),
            enter_speed=float(rospy.get_param('~standstill_enter_speed',.010)),
            exit_speed=float(rospy.get_param('~standstill_exit_speed',.015)),
            position_span=float(rospy.get_param('~standstill_position_span',.010)),
            heading_span=float(rospy.get_param('~standstill_heading_span',.015)),
            min_samples=rospy.get_param('~standstill_min_samples',3))
        validate(self.speeds);validate(self.steering);validate(self.reverse_steering)
        self.L=float(rospy.get_param('~wheelbase'));self.T=float(rospy.get_param('~front_track'))
        self.Tr=float(rospy.get_param('~rear_track'));self.radius=float(rospy.get_param('~wheel_radius'))
        self.K=float(rospy.get_param('~ticks_per_meter'));self.middle=float(rospy.get_param('~legacy_middle'));self.scale=float(rospy.get_param('~legacy_scale'))
        self.timeout=float(rospy.get_param('~command_timeout',.4));self.fresh=float(rospy.get_param('~feedback_timeout',.3))
        self.window=float(rospy.get_param('~velocity_window',.25));self.max_v=float(rospy.get_param('~max_feedback_speed',.5))
        if not all(finite(x) and x>0 for x in (self.L,self.T,self.Tr,self.radius,self.K,self.scale,self.timeout,self.fresh,self.window,self.max_v)):
            raise ValueError('invalid bridge geometry or timing')
        self.reverse=bool(rospy.get_param('~allow_reverse',False));self.enabled=bool(rospy.get_param('~enable_drive',False))
        self.hold_on_fault=bool(rospy.get_param('~hold_steering_on_fault',True))
        self.pose_timeout=float(rospy.get_param('~lidar_velocity_timeout',.4))
        self.pulse_gap=float(rospy.get_param('~pulse_max_update_gap',4.))
        if not finite(self.pose_timeout) or self.pose_timeout<=0 or not finite(self.pulse_gap) or self.pulse_gap<=0:
            raise ValueError('invalid feedback timeout')
        self.velocity_odom_frame=rospy.get_param('~velocity_odom_frame','odom')
        self.velocity_base_frame=rospy.get_param('~velocity_base_frame','base_footprint')
        self.tf=tf.TransformListener()
        self.pose_monitor=PoseVelocity(self.window,self.max_v,
            moving_window=float(rospy.get_param('~moving_velocity_window',.25)))
        self.pulse_monitor=PulseMonitor(self.K,self.max_v,self.pulse_gap)
        self.velocity_stamp=None
        self.enable_prediction=bool(rospy.get_param('~enable_speed_prediction',True))
        self.prediction_mode=rospy.get_param('~prediction_mode','observer')
        if self.prediction_mode not in ('observer','command_reference'):raise ValueError('invalid prediction_mode')
        self.fault_steering=None;self.last_sent_steering=None
        self.master=rosgraph.Master(rospy.get_name());self.owner_ok=False;self.owner_reason='checking command ownership'
        self.samples=deque();self.previous_tick=None;self.total=0.;self.steer=None;self.steer_time=None;self.tick_time=None;self.velocity=None;self.last_cmd=None;self.target=None
        model=dict(version=1,wheelbase=self.L,front_track=self.T,wheel_radius=self.radius)
        for key in ('acceleration','braking','steer_rate'):
            val=float(rospy.get_param('~'+key))
            if not finite(val) or val<=0:raise ValueError('invalid actuator model '+key)
            model[key]=val
        measured_response=rospy.get_param('~steering_response',None)
        if measured_response is not None:
            # Navigator validates and models the measured response. Keep the
            # original version-1 actuator fields for existing integrations.
            model['steering_response']=measured_response
            reverse_response=rospy.get_param('~steering_response_reverse',None)
            if reverse_response is not None:model['steering_response_reverse']=reverse_response
        rospy.set_param('/vehicle/actuator_model',model)
        self.speed_observer=SpeedObserver(model['acceleration'],model['braking'],self.max_v,
            float(rospy.get_param('~prediction_correction_gain',.4)))
        rospy.set_param('/vehicle/calibration_info',dict(steering='piecewise measured steering samples' if len(self.steering)>2 else 'linear measured extremes',steering_samples=len(self.steering),feedback='estimated individual joints: centre steering and lidar rear-axle velocity',velocity_source='lidar_odometry',wheel_velocity_estimated=True,reverse_calibrated=rospy.has_param("~reverse_steering_table"),reverse_steering_samples=len(self.reverse_steering),reverse_straight_raw=interpolate(0.,[(p[1],p[0]) for p in self.reverse_steering]),reverse_calibration_scope="piecewise curvature; low-speed samples except maximum left; approximate user-confirmed zero",dynamics_calibrated=rospy.get_param('~dynamics_calibrated',False)))
        self.out=rospy.Publisher('/cmd_vel',Twist,queue_size=1) if self.enabled else None
        self.preview=rospy.Publisher('/vehicle/legacy_cmd_preview',Twist,queue_size=1)
        self.feedback=rospy.Publisher('/vehicle/joint_states',JointState,queue_size=1)
        self.status=rospy.Publisher('/vehicle/bridge_status',String,queue_size=1,latch=True)
        self.motion=rospy.Publisher('/vehicle/motion_estimate',String,queue_size=1)
        rospy.Subscriber('/steering_raw',Int32,self.on_steer,queue_size=10)
        rospy.Subscriber('/wheel_ticks_raw',Int64,self.on_ticks,queue_size=20)
        rospy.Subscriber('/vehicle/cmd_vel',Twist,self.on_command,queue_size=1)
        rospy.Subscriber('/vehicle/steering_request',String,self.on_steering_request,queue_size=1)
        self.ownership=rospy.Timer(rospy.Duration(.25),self.check_ownership)
        self.timer=rospy.Timer(rospy.Duration(.05),self.on_timer)
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn('Vehicle bridge enable_drive=%s; reverse=%s; dynamics are provisional',self.enabled,self.reverse)

    def check_ownership(self,event):
        try:
            pubs=self.master.getSystemState()[0]
            others=[n for topic,nodes in pubs if topic in ('/cmd_vel','/joy') for n in nodes if n!=rospy.get_name()]
            with self.lock:self.owner_ok=not others;self.owner_reason='other publishers: '+','.join(others) if others else ''
        except Exception as e:
            with self.lock:self.owner_ok=False;self.owner_reason=str(e)

    def on_steer(self,m):
        with self.lock:
            raw=float(m.data)
            if finite(raw) and self.feedback_raw_min<=raw<=self.feedback_raw_max:
                self.steer_invalid=False
                self.steer=raw;self.steer_time=clock()
            else:self.steer_invalid=True;self.feedback_extrapolated=False

    def on_ticks(self,m):
        with self.lock:
            now=clock();self.tick_time=now
            self.pulse_monitor.update(m.data,now)
            self.total=self.pulse_monitor.total

    def update_velocity(self,now):
        self.raw_velocity=None
        try:
            stamp=self.tf.getLatestCommonTime(self.velocity_odom_frame,self.velocity_base_frame)
            age=(rospy.Time.now()-stamp).to_sec()
            if not finite(age) or age<0 or age>self.pose_timeout:
                self.velocity=None;self.standstill.reset('STALE_OR_INVALID');return
            p,q=self.tf.lookupTransform(self.velocity_odom_frame,self.velocity_base_frame,stamp)
            yaw=tf.transformations.euler_from_quaternion(q)[2]
            self.pose_monitor.update(stamp.to_sec(),(p[0],p[1],yaw),now)
            self.velocity=self.pose_monitor.velocity
            self.raw_velocity=self.velocity
            self.velocity_stamp=stamp
            if self.pose_monitor.received is None or now-self.pose_monitor.received>self.pose_timeout:self.velocity=None
            if self.standstill.update(now,self.command_speed,self.velocity,stamp.to_sec(),(p[0],p[1],yaw),fresh=age<=min(self.fresh,self.pose_timeout)):
                self.velocity=0.
        except (tf.Exception,ValueError,TypeError):
            self.velocity=None;self.standstill.reset('STALE_OR_INVALID')

    def steering_table(self, direction):
        return self.reverse_steering if direction < 0 else self.steering

    def publish_feedback(self):
        direction = (1 if self.velocity > 0 else -1) if abs(self.velocity) > .01 else self.direction
        try:angles,wheels,k=joints(self.steer,self.velocity,self.steering_table(direction),self.L,self.T,self.Tr,self.radius,self.feedback_raw_min,self.feedback_raw_max)
        except ValueError:self.velocity=None;return
        j=JointState();j.header.stamp=self.velocity_stamp;j.header.frame_id=self.velocity_base_frame
        j.name=['front_left_steer_joint','front_right_steer_joint','rear_left_wheel_joint','rear_right_wheel_joint']
        j.position=angles+[0.,0.];j.velocity=[0.,0.]+wheels
        self.feedback.publish(j)
        self.feedback_extrapolated=not self.steering_table(direction)[0][0]<=self.steer<=self.steering_table(direction)[-1][0]

    def on_steering_request(self,m):
        with self.lock:
            try:
                d=json.loads(m.data);angle=float(d['angle']);direction=int(d['direction'])
                if not finite(angle) or abs(angle)>.8 or direction not in (-1,1):raise ValueError('invalid steering request')
                self.steering_request=(clock(),angle,direction)
            except (ValueError,TypeError,KeyError):self.steering_request=None

    def on_command(self,m):
        with self.lock:
            self.last_cmd=clock()
            self.command_speed=m.linear.x
            if abs(m.linear.x)>1e-6:self.standstill.reset('COMMAND_MOVING')
            try:
                direction=(1 if m.linear.x > 0 else -1) if abs(m.linear.x)>1e-6 else self.direction
                target=command(m.linear.x,m.angular.z,self.speeds,self.steering_table(direction),self.middle,self.scale,self.reverse)
                self.target=target;self.direction=direction
                if abs(m.linear.x)>1e-6:self.steering_request=None
            except ValueError as e:self.target=None;rospy.logwarn_throttle(1.,str(e))

    def fault_stop(self):
        # Latch the angle at fault entry, avoiding feedback drift during the stop.
        msg=Twist()
        if self.hold_on_fault:
            if self.fault_steering is None:
                raw=self.steer
                self.fault_steering=max(-1.,min(1.,(self.middle-raw)/self.scale)) if raw is not None and finite(raw) and self.feedback_raw_min<=raw<=self.feedback_raw_max else self.last_sent_steering
            if self.fault_steering is not None:msg.angular.z=self.fault_steering
        return msg

    def on_timer(self,event):
        with self.lock:
            now=clock();reason='READY'
            self.update_velocity(now)
            if not self.owner_ok:reason='CONTROL_CONFLICT: '+self.owner_reason
            elif self.steer_invalid:reason='INVALID_STEERING_FEEDBACK'
            elif self.tick_time is None or self.steer_time is None or now-self.tick_time>self.fresh or now-self.steer_time>self.fresh or self.velocity is None:reason='STALE_FEEDBACK'
            elif self.last_cmd is None or now-self.last_cmd>self.timeout:reason='NO_FRESH_COMMAND'
            elif self.target is None:reason='INVALID_COMMAND'
            if not self.steer_invalid and self.velocity is not None and self.steer_time is not None and now-self.steer_time<=self.fresh:
                self.publish_feedback()
                if self.velocity is None:reason='INVALID_FEEDBACK'
            msg=Twist()
            if reason=='READY':
                msg.linear.x,msg.angular.z=self.target[:2]
                req=self.steering_request
                if req is not None and now-req[0]<=self.timeout and abs(msg.linear.x)<1e-6:
                    # Hold measured steering while braking; never first return to straight.
                    raw=self.steer
                    if abs(self.velocity)<=.005:
                        k=math.tan(req[1])/self.L
                        table=self.steering_table(req[2])
                        raw=interpolate(k,[(p[1],p[0]) for p in table])
                        self.direction=req[2]
                    msg.angular.z=max(-1.,min(1.,(self.middle-raw)/self.scale))
                self.fault_steering=None
            else:msg=self.fault_stop()
            self.preview.publish(msg)
            # When another controller owns the chassis, remain silent.
            if self.out is not None and self.owner_ok and (not self.hold_on_fault or self.steer is not None or self.last_sent_steering is not None):
                self.out.publish(msg);self.last_sent_steering=msg.angular.z
            prediction=None
            if self.enable_prediction and self.enabled and reason=='READY' and self.pose_monitor.measurement_stamp is not None:
                # Invert the actual sent throttle through the calibration table.
                target=math.copysign(interpolate(abs(msg.linear.x),[(p[1],p[0]) for p in self.speeds]),msg.linear.x) if msg.linear.x else 0.
                delay=rospy.Time.now().to_sec()-self.pose_monitor.measurement_stamp
                if self.prediction_mode == 'command_reference':
                    self.speed_observer.reset();self.speed_observer.state='COMMAND_REFERENCE'
                    prediction=target
                else:
                    prediction=self.speed_observer.update(now,target,self.velocity,now-delay,self.velocity_stamp.to_sec())
            else:self.speed_observer.reset()
            self.motion.publish(String(data=json.dumps(dict(stamp=rospy.Time.now().to_sec(),
                valid=prediction is not None,predicted_velocity=prediction,observed_velocity=self.velocity,
                raw_observed_velocity=self.raw_velocity,stationary_confirmed=self.standstill.confirmed,
                estimated=True,state=self.speed_observer.state))))
            self.status.publish(String(data=json.dumps(dict(state=reason,drive_enabled=self.enabled,steering_calibration_direction='reverse' if self.direction<0 else 'forward',measured_speed=self.velocity,raw_measured_speed=self.raw_velocity,stationary_confirmed=self.standstill.confirmed,standstill_state=self.standstill.state,steering_raw=self.steer,steering_feedback_extrapolated=self.feedback_extrapolated,velocity_source='lidar_odometry',wheel_velocity_estimated=True,pulse_state=self.pulse_monitor.state,pulse_distance=self.total,pulse_average_speed=self.pulse_monitor.average))))

    def shutdown(self):
        if self.out is not None and self.owner_ok and (not self.hold_on_fault or self.steer is not None or self.last_sent_steering is not None):
            with self.lock:msg=self.fault_stop()
            for i in range(5):self.out.publish(msg);time.sleep(.02)

if __name__=='__main__':
    rospy.init_node('vehicle_bridge');Bridge();rospy.spin()
