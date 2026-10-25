"""Timestamped signed pulse buffering and rear-axle motion constraints, no ROS."""
import bisect
import math
from collections import deque


def finite(v): return not (math.isnan(v) or math.isinf(v))


def wrap(a): return math.atan2(math.sin(a),math.cos(a))


class PulseBuffer(object):
    def __init__(self,ticks_per_meter=10000.,max_gap=.15,max_speed=1.,window=15.):
        for v in (ticks_per_meter,max_gap,max_speed,window):
            if not finite(v) or v<=0: raise ValueError('Invalid encoder configuration')
        self.scale=ticks_per_meter;self.gap=max_gap;self.speed=max_speed;self.window=window
        self.samples=deque();self.epoch=None;self.generation=0

    def add(self,stamp,ticks,epoch):
        if not all(finite(v) for v in (stamp,ticks)) or stamp<=0 or ticks!=int(ticks) or abs(ticks)>=2**53:
            raise ValueError('Invalid pulse sample')
        if self.epoch!=epoch:
            self.samples.clear();self.epoch=epoch;self.generation+=1
        if self.samples:
            previous,count=self.samples[-1]
            if stamp<=previous: raise ValueError('Out-of-order pulse sample')
            if abs(ticks-count)/self.scale>self.speed*(stamp-previous)+2/self.scale:
                self.samples.clear();self.generation+=1
                self.samples.append((stamp,ticks));raise ValueError('Encoder discontinuity')
        self.samples.append((stamp,ticks))
        while len(self.samples)>2 and self.samples[1][0]<stamp-self.window: self.samples.popleft()

    def value(self,stamp):
        data=list(self.samples);times=[x[0] for x in data]
        i=bisect.bisect_left(times,stamp)
        if i<len(data) and abs(times[i]-stamp)<1e-8: return data[i][1]/self.scale
        if i==0 or i==len(data): raise ValueError('Pulse data does not bracket scan timestamp')
        t0,c0=data[i-1];t1,c1=data[i]
        if t1-t0>self.gap: raise ValueError('Pulse stream gap')
        return (c0+(c1-c0)*(stamp-t0)/(t1-t0))/self.scale

    def delta(self,start,end):
        if end<=start: raise ValueError('Invalid encoder interval')
        # Gaps inside the interval must not disappear through endpoint interpolation.
        data=list(self.samples)
        for a,b in zip(data,data[1:]):
            if b[0]>start and a[0]<end and b[0]-a[0]>self.gap: raise ValueError('Pulse stream gap')
        return self.value(end)-self.value(start)


def wheel_prediction(previous,yaw_delta,distance,base_offset=.31):
    """Integrate signed rear-centre arc using scan yaw, then restore base point."""
    yaw=previous[2];angle=wrap(yaw_delta)
    rear=(previous[0]-base_offset*math.cos(yaw),previous[1]-base_offset*math.sin(yaw))
    chord=distance*(math.sin(angle/2)/(angle/2) if abs(angle)>1e-9 else 1.)
    direction=yaw+angle/2;new_yaw=wrap(yaw+angle)
    return (rear[0]+chord*math.cos(direction)+base_offset*math.cos(new_yaw),
            rear[1]+chord*math.sin(direction)+base_offset*math.sin(new_yaw),new_yaw)


def distance_prior(previous,distance,base_offset,sigma):
    return dict(previous=previous,distance=distance,base_offset=base_offset,sigma=sigma)


def prior_residual_jacobian(pose,prior):
    """Rear-centre longitudinal displacement, not a second heading observation."""
    old=prior['previous'];offset=prior['base_offset'];d=wrap(pose[2]-old[2]);mid=old[2]+d/2
    c,s=math.cos(mid),math.sin(mid)
    dx=pose[0]-offset*math.cos(pose[2])-(old[0]-offset*math.cos(old[2]))
    dy=pose[1]-offset*math.sin(pose[2])-(old[1]-offset*math.sin(old[2]))
    factor=math.sin(d/2)/(d/2) if abs(d)>1e-7 else 1-d*d/24
    derivative=(.5*(d/2*math.cos(d/2)-math.sin(d/2))/(d/2)**2 if abs(d)>1e-7 else -d/12)
    residual=dx*c+dy*s-prior['distance']*factor
    jy=offset*math.sin(pose[2])*c-offset*math.cos(pose[2])*s+.5*(-dx*s+dy*c)-prior['distance']*derivative
    return residual,(c,s,jy)


class PulseSimulator(object):
    def __init__(self,ticks_per_meter=10000.,scale_error=0.,rear_offset=.31,max_speed=2.):
        if ticks_per_meter<=0 or not all(finite(v) for v in (ticks_per_meter,scale_error,rear_offset,max_speed)) or 1+scale_error<=0:
            raise ValueError('Invalid simulated encoder configuration')
        self.scale=ticks_per_meter;self.error=scale_error;self.offset=rear_offset;self.max_speed=max_speed
        self.last=None;self.distance=0.;self.epoch=0

    def update(self,stamp,pose):
        rear=(pose[0]-self.offset*math.cos(pose[2]),pose[1]-self.offset*math.sin(pose[2]))
        if self.last is not None:
            t,p,r=self.last;dt=stamp-t;dx,dy=rear[0]-r[0],rear[1]-r[1]
            if dt<=0 or math.hypot(dx,dy)>self.max_speed*dt+.02:
                self.distance=0.;self.epoch+=1
            else:
                angle=wrap(pose[2]-p[2]);mid=p[2]+angle/2
                chord=dx*math.cos(mid)+dy*math.sin(mid)
                factor=math.sin(angle/2)/(angle/2) if abs(angle)>1e-8 else 1.
                self.distance+=chord/factor*(1+self.error)
        self.last=(stamp,pose,rear)
        return int(round(self.distance*self.scale))
