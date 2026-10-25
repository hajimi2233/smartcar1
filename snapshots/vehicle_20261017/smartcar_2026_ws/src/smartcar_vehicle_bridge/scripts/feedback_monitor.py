"""Measured pose velocity and delayed cumulative-count diagnostics (Python 2/3)."""
from __future__ import division
import math
from collections import deque

def finite(x):return not (math.isnan(x) or math.isinf(x))
def wrap(x):return math.atan2(math.sin(x),math.cos(x))

class PoseVelocity(object):
    def __init__(self,window=.25,max_speed=.5,slack=.02,min_span=.1,moving_window=None,motion_threshold=.03):
        self.window=window;self.max_speed=max_speed;self.slack=slack;self.min_span=min_span
        self.samples=deque();self.velocity=None;self.received=None;self.state='WAIT_POSE'
        self.measurement_stamp=None
        self.moving_window=window if moving_window is None else moving_window
        if not min_span<=self.moving_window<=window:raise ValueError('invalid moving velocity window')
        self.motion_threshold=motion_threshold;self.last_motion=None

    def update(self,stamp,pose,received):
        if not all(finite(x) for x in (stamp,received)+tuple(pose)):
            self.samples.clear();self.velocity=None;self.state='INVALID_POSE';return
        if self.samples:
            t,p=self.samples[-1];dt=stamp-t
            if dt==0:return  # Repeated TF must not refresh the feedback deadline.
            if dt<0:
                self.samples.clear();self.velocity=None;self.received=None;self.state='CLOCK_RESET'
                return
            distance=math.hypot(pose[0]-p[0],pose[1]-p[1])
            if distance>self.max_speed*dt+self.slack:
                self.samples.clear();self.samples.append((stamp,tuple(pose)))
                self.velocity=None;self.received=None;self.state='POSE_JUMP';return
        self.samples.append((stamp,tuple(pose)));self.received=received
        while len(self.samples)>2 and stamp-self.samples[1][0]>=self.window:self.samples.popleft()
        recent=list(self.samples)
        start=0
        while start+1<len(recent)-1 and stamp-recent[start+1][0]>=self.moving_window:start+=1
        t0,p0=recent[start];span=stamp-t0
        yaw0=p0[2]+.5*wrap(pose[2]-p0[2])
        short_speed=((pose[0]-p0[0])*math.cos(yaw0)+(pose[1]-p0[1])*math.sin(yaw0))/span if span>=self.min_span else 0.
        if abs(short_speed)>self.motion_threshold:self.last_motion=stamp
        if self.last_motion is not None and 0<=stamp-self.last_motion<=self.window:recent=recent[start:]
        t,p=recent[0];dt=stamp-t
        if dt<self.min_span:
            self.velocity=None;self.state='WAIT_WINDOW';return
        yaw=p[2]+.5*wrap(pose[2]-p[2])
        # Fit every measured pose in the window instead of differentiating two noisy endpoints.
        points=[(s-t,(q[0]-p[0])*math.cos(yaw)+(q[1]-p[1])*math.sin(yaw)) for s,q in recent]
        mean_t=sum(q[0] for q in points)/len(points)
        self.measurement_stamp=t+mean_t
        mean_s=sum(q[1] for q in points)/len(points)
        denominator=sum((q[0]-mean_t)**2 for q in points)
        velocity=sum((q[0]-mean_t)*(q[1]-mean_s) for q in points)/denominator
        if not finite(velocity) or abs(velocity)>self.max_speed:
            self.velocity=None;self.state='INVALID_SPEED';return
        self.velocity=velocity;self.state='READY'

class PulseMonitor(object):
    def __init__(self,ticks_per_meter,max_speed=.5,max_update_gap=4.,slack=.02):
        self.K=ticks_per_meter;self.max_speed=max_speed;self.max_gap=max_update_gap;self.slack=slack
        self.previous=None;self.changed=None;self.received=None;self.total=0.;self.average=None
        self.state='WAIT_TICKS';self.delta=0;self.elapsed=None

    def update(self,ticks,now):
        cur=int(ticks)&0xffffffff;self.received=now
        if self.previous is None:
            self.previous=cur;self.changed=now;self.state='BASELINE';return
        delta=((cur-self.previous+0x80000000)&0xffffffff)-0x80000000
        self.delta=delta
        if delta==0:
            # Equal count reports prove communication, not that the car is stationary.
            self.state='UNCHANGED';return
        elapsed=now-self.changed;self.elapsed=elapsed
        distance=delta/self.K
        self.previous=cur;self.changed=now
        if elapsed<=0 or abs(distance)>self.max_speed*min(elapsed,self.max_gap)+self.slack:
            self.average=None;self.state='INVALID_INCREMENT';return
        self.total+=distance;self.average=distance/elapsed
        self.state='DELAYED_UPDATE' if elapsed>.3 else 'UPDATED'
