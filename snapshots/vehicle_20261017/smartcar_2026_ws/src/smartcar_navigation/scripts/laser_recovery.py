"""Bounded recovery against raw laser hits AND observed free space. Py2/3."""
import math,time
import numpy as np
from nav_config import P
from ackermann_core import advance,finite,front_position
from inspection_depth import blocked as depth_blocked

class LiveScanGrid(object):
    """Static-map walls are ignored here; mission constraints remain hard.

    Invalid/missing rays are unknown, not free. The current physical vehicle
    footprint is known occupied-by-vehicle space; freshly observed hits inside
    it still reject recovery. All newly swept space requires valid adjacent rays.
    """
    def __init__(self,base,scan,laser_pose,vehicle_pose,origin,limit):
        self.base=base.snapshot();self.margin=base.margin;self.res=.01
        self.half_width=base.half_width;self.region=list(base.region);self.depth_rules=list(base.depth_rules)
        self.laser=tuple(laser_pose);self.vehicle=tuple(vehicle_pose)
        self.origin=tuple(origin);self.limit=limit;self.last_reason=''
        self.created=time.time();self.capture_age=0.
        self.ranges=np.asarray(scan.ranges,dtype=float).copy()
        self.amin=float(scan.angle_min);self.inc=float(scan.angle_increment)
        if (not finite(laser_pose+vehicle_pose) or len(self.ranges)<20 or not 0<self.inc<=.1
                or not 0<=scan.range_min<scan.range_max):
            raise ValueError('invalid laser recovery geometry')
        finite_ranges=np.isfinite(self.ranges)
        clean=np.where(finite_ranges,self.ranges,0.)
        self.valid=finite_ranges&(clean>=scan.range_min)&(clean<scan.range_max)
        self.ranges=np.where(self.valid,clean,0.)
        if int(self.valid.sum())<20:raise ValueError('insufficient raw recovery rays')
        self.full_circle=(len(self.ranges)+1)*self.inc>=2.*math.pi
        angles=self.amin+np.arange(len(self.ranges))*self.inc+laser_pose[2]
        self.hits=np.column_stack((laser_pose[0]+self.ranges[self.valid]*np.cos(angles[self.valid]),
                                   laser_pose[1]+self.ranges[self.valid]*np.sin(angles[self.valid])))
        self._stencils={}
    def _rectangle(self,pose,points,a,b):
        c,s=math.cos(pose[2]),math.sin(pose[2])
        dx=points[:,0]-pose[0];dy=points[:,1]-pose[1]
        return (np.abs(c*dx+s*dy-P['collision_center_x'])<=a)&(np.abs(-s*dx+c*dy)<=b)
    def enable_depth_escape(self,limit,padding=0.,previous=None):
        from execution_patch import DepthEscape
        self.depth_escape=DepthEscape(self.depth_rules,front_position(self.origin),limit,padding)
        self.depth_escape.update_observed(front_position(self.vehicle),previous)
        return self.depth_escape
    def _samples(self,pose,extra):
        # A half sample-diagonal covers the space between the stencil points.
        pad=self.margin+extra+P['laser_recovery_grid_step']*.707107
        a=P['collision_half_length']+pad;b=self.half_width+pad
        key=round(extra,5)
        if key not in self._stencils:
            xs=np.linspace(P['collision_center_x']-a,P['collision_center_x']+a,
                           int(math.ceil(2*a/P['laser_recovery_grid_step']))+1)
            ys=np.linspace(-b,b,int(math.ceil(2*b/P['laser_recovery_grid_step']))+1)
            xx,yy=np.meshgrid(xs,ys);self._stencils[key]=np.column_stack((xx.ravel(),yy.ravel()))
        q=self._stencils[key];c,s=math.cos(pose[2]),math.sin(pose[2])
        return np.column_stack((pose[0]+c*q[:,0]-s*q[:,1],pose[1]+s*q[:,0]+c*q[:,1])),a,b
    def collision(self,pose,detailed=False,extra=0.):
        reason=None
        if not finite(pose):reason='INVALID_POSE'
        elif math.hypot(pose[0]-self.origin[0],pose[1]-self.origin[1])>self.limit+1e-8:reason='RECOVERY_DISTANCE'
        elif (self.depth_escape.collision(front_position(pose),extra) if getattr(self,'depth_escape',None)
              else depth_blocked(front_position(pose),self.depth_rules,extra+getattr(self,'depth_margin',0.))):reason='INSPECTION_DEPTH'
        elif self.base.region_blocked(pose,extra):reason='REGION'
        if reason:self.last_reason=reason;return {reason:None}
        points,a,b=self._samples(pose,extra)
        inside=self._rectangle(pose,self.hits,a,b)
        if np.any(inside):
            hit=self.hits[np.flatnonzero(inside)[0]];self.last_reason='RAW_LIVE_SCAN'
            return {'RAW_LIVE_SCAN':(float(hit[0]),float(hit[1]))}
        known=self._rectangle(self.vehicle,points,P['collision_half_length'],self.half_width)
        pts=points[~known]
        if len(pts):
            dx=pts[:,0]-self.laser[0];dy=pts[:,1]-self.laser[1]
            distance=np.hypot(dx,dy);angle=(np.arctan2(dy,dx)-self.laser[2]-self.amin)%(2*math.pi)
            index=angle/self.inc;lo=np.floor(index).astype(int);hi=lo+1
            if self.full_circle:
                lo=lo%len(self.ranges);hi=hi%len(self.ranges);covered=np.ones(len(lo),dtype=bool)
            else:
                covered=(lo>=0)&(hi<len(self.ranges));lo=np.clip(lo,0,len(self.ranges)-1);hi=np.clip(hi,0,len(self.ranges)-1)
            clearance=np.minimum(np.minimum(self.ranges[lo],self.ranges[hi]),P['laser_recovery_range'])-.01
            covered&=self.valid[lo]&self.valid[hi]&(distance<=clearance)
            if not np.all(covered):
                self.last_reason='UNOBSERVED_LASER_SPACE';return {'UNOBSERVED_LASER_SPACE':None}
        self.last_reason='';return {}
    def free(self,pose):return not self.collision(pose)
    def blocked_detail(self,pose,distance=0.,curvature=0.):
        return 'raw laser recovery sources='+','.join(sorted(self.collision(pose,detailed=True)))
    def arc(self,pose,distance,curvature):
        count=max(1,int(math.ceil(abs(distance)/.01)))
        step=distance/count;points=[]
        radius=math.hypot(P['collision_center_x']+P['collision_half_length'],self.half_width)
        sweep=abs(step)*.5*(1.+radius*abs(curvature))
        for unused in range(count):
            if self.collision(pose,extra=sweep):return None
            next_pose=advance(pose,step,curvature)
            if getattr(self,'depth_escape',None) and not self.depth_escape.monotonic(front_position(pose),front_position(next_pose)):
                return None
            pose=next_pose
            if self.collision(pose,extra=sweep):return None
            points.append(pose)
        return points

def recovery_path(raw_grid,map_grid,pose,actual_steer,radius,deadline,cancel=lambda:False):
    """Finite local escape probes; an endpoint must rejoin normal map clearance."""
    measured=max(-1./max(radius,P['local_right_min_radius']),
                 min(1./max(radius,P['local_left_min_radius']),math.tan(actual_steer)/P['wheelbase']))
    choices=(measured,0.,1./max(radius,P['local_left_min_radius']),
             -1./max(radius,P['local_right_min_radius']))
    curves=[]
    for k in choices:
        if not any(abs(k-v)<1e-6 for v in curves):curves.append(k)
    candidates=[]
    escape=getattr(raw_grid,'depth_escape',None)
    directions=(-1,) if escape is not None and escape.budgets else (-1,1)
    for direction in directions:
        for distance in (.05,.10,.15,.20,P['laser_recovery_distance']):
            if distance>P['laser_recovery_distance']:continue
            for curvature in curves:
                if cancel() or time.time()>deadline:return None
                end=advance(pose,direction*distance,curvature)
                if map_grid.collision(end,extra=P['safety_stop_extra']+.01):continue
                candidates.append((distance+.04*abs(curvature-measured),direction,distance,curvature))
    for unused,direction,distance,curvature in sorted(candidates):
        if cancel() or time.time()>deadline:return None
        points=raw_grid.arc(pose,direction*distance,curvature)
        if points is not None:
            return [tuple(pose)+(direction,curvature)]+[tuple(p)+(direction,curvature) for p in points]
    return None
