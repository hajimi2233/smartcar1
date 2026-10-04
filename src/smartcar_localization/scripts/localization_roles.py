"""Experimental localization policies. Receives measurements, NEVER ground truth.

All modes use the same scan/scan-odometry and map targets. AMCL is a separately
running ROS particle filter. The role policy preserves strong selected-wall
constraints and asks full-map matching only to fill missing directions.
"""
import math
import numpy as np
from wall_features import compose,inverse,match
from wheel_distance import wheel_prediction,distance_prior,wrap

MODES=('wall_lidar','wall_weighted','amcl_scan','amcl_wheel','role_split','full_map')


class Policies(object):
    def __init__(self,initial,odom,stamp,modes=MODES):
        self.modes=tuple(modes)
        self.poses={mode:tuple(initial) for mode in self.modes}
        self.last_odom=odom;self.stamp=stamp;self.amcl_seen={};self.failures={mode:0 for mode in self.modes}
        self.valid={mode:False for mode in self.modes}

    @staticmethod
    def scan_prediction(pose,old_odom,odom):
        return compose(pose,compose(inverse(old_odom),odom))

    def update(self,stamp,odom,points,normals,walls,all_walls,distance,amcl):
        predictions={};details={};old=dict(self.poses)
        for mode in self.modes:
            p=self.scan_prediction(old[mode],self.last_odom,odom)
            use_wheel=mode in ('wall_weighted','amcl_wheel','role_split') and distance is not None
            if use_wheel:p=wheel_prediction(old[mode],wrap(odom[2]-self.last_odom[2]),distance,.31)
            predictions[mode]=p
        for mode in ('wall_lidar','wall_weighted','full_map','role_split'):
            if mode not in predictions:continue
            predicted=predictions[mode]
            prior=(distance_prior(old[mode],distance,.31,.005+.05*abs(distance))
                   if mode=='wall_weighted' and distance is not None else None)
            targets=all_walls if mode=='full_map' else walls
            fitted,info=match(points,normals,predicted,targets,motion_prior=prior)
            state='MAP' if fitted is not None else 'PREDICT_ONLY'
            if mode=='role_split':
                if fitted is not None and info.get('map_rank',0)<3:
                    # Do not blend every coordinate. Selected walls retain their
                    # constrained directions; full map supplies only their nullspace.
                    global_fit,global_info=match(points,normals,fitted,all_walls)
                    if (global_fit is None or global_info.get('map_rank',0)<3) and amcl is not None:
                        anchor,anchor_odom,anchor_stamp,variance=amcl
                        if variance<.15 and stamp-anchor_stamp<2.:
                            proposal=self.scan_prediction(anchor,anchor_odom,odom)
                            confirmed,check=match(points,normals,proposal,all_walls)
                            if confirmed is not None and check.get('map_rank')==3:
                                global_fit=confirmed;state='AMCL_NULLSPACE_CONFIRMED'
                    if global_fit is not None:
                        values,vectors=np.linalg.eigh(np.asarray(info['covariance']))
                        weak=vectors[:,np.argsort(values)[-(3-info['map_rank']):]]
                        delta=np.asarray(global_fit)-np.asarray(fitted);delta[2]=wrap(delta[2])
                        fitted=tuple(np.asarray(fitted)+np.dot(weak,np.dot(weak.T,delta)))
                        if state!='AMCL_NULLSPACE_CONFIRMED':state='WALL_PLUS_MAP_NULLSPACE'
                elif fitted is None:
                    fitted,info=match(points,normals,predicted,all_walls)
                    if fitted is not None:state='FULL_MAP_FALLBACK'
                if fitted is None and amcl is not None and self.failures[mode]>=2:
                    anchor,anchor_odom,anchor_stamp,variance=amcl
                    # AMCL proposes a coarse hypothesis; a successful map check must
                    # confirm it before it replaces the local track.
                    proposal=self.scan_prediction(anchor,anchor_odom,odom)
                    candidate,check=match(points,normals,proposal,all_walls)
                    if candidate is not None and check.get('map_rank')==3 and variance<.15 and stamp-anchor_stamp<2.:
                        fitted,info=candidate,check;state='AMCL_RECOVERY_CONFIRMED'
            if fitted is not None:
                self.poses[mode]=fitted;self.failures[mode]=0;self.valid[mode]=True
            else:
                self.poses[mode]=predicted;self.failures[mode]+=1
            details[mode]=dict(state=state,valid=self.valid[mode] and self.failures[mode]<=20,
                               reason=info.get('reason'),failures=self.failures[mode])
        for mode in ('amcl_scan','amcl_wheel'):
            if mode not in predictions:continue
            p=predictions[mode];state='PREDICT_ONLY'
            if amcl is not None:
                anchor,anchor_odom,anchor_stamp,variance=amcl
                if anchor_stamp>self.amcl_seen.get(mode,-1):
                    # Transport the delayed AMCL anchor using scan odometry only
                    # across that delay. Subsequent short increments use each mode's predictor.
                    p=self.scan_prediction(anchor,anchor_odom,odom)
                    self.amcl_seen[mode]=anchor_stamp;state='AMCL_ANCHOR';self.valid[mode]=True
            self.poses[mode]=p
            age=stamp-self.amcl_seen.get(mode,-1e6)
            details[mode]=dict(state=state,valid=self.valid[mode] and age<=2.,anchor_age=age)
        self.last_odom=odom;self.stamp=stamp
        return dict(self.poses),details
