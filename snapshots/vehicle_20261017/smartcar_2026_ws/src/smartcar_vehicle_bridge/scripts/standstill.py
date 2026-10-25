"""Fresh, multi-frame stop evidence; no zero-command-only assumption (Py2/3)."""
import math
from collections import deque

class Standstill(object):
    def __init__(self,window=.35,enter_speed=.010,exit_speed=.015,position_span=.010,heading_span=.015,min_samples=3):
        values=(window,enter_speed,exit_speed,position_span,heading_span)
        if (any(math.isnan(v) or math.isinf(v) or v<=0 for v in values)
                or not enter_speed<exit_speed<=.020 or enter_speed>.012
                or not .2<=window<=2. or position_span>.020 or heading_span>.035
                or min_samples<3 or int(min_samples)!=min_samples):
            raise ValueError('invalid standstill confirmation parameters')
        self.window=window;self.enter_speed=enter_speed;self.exit_speed=exit_speed
        self.position_span=position_span;self.heading_span=heading_span;self.min_samples=min_samples;self.reset()

    def reset(self,state='WAIT_STABILITY'):
        self.motion_proven=(state=='POSE_MOVING' or (state not in ('COMMAND_MOVING','CLOCK_RESET','WAIT_STABILITY') and getattr(self,'motion_proven',False)))
        self.samples=deque();self.confirmed=False;self.last_stamp=None;self.state=state
        self.anchor=None;self.above_enter_since=None

    def update(self,now,command_speed,observed,stamp,pose,fresh=True):
        if pose is None or len(pose)!=3:self.reset('STALE_OR_INVALID');return False
        values=(now,command_speed,observed,stamp)+tuple(pose) if None not in (now,command_speed,observed,stamp) else ()
        if not values or not fresh or any(math.isnan(v) or math.isinf(v) for v in values):
            self.reset('STALE_OR_INVALID');return False
        if abs(command_speed)>1e-6:self.reset('COMMAND_MOVING');return False
        threshold=self.exit_speed if self.confirmed else min(self.enter_speed,.005) if self.motion_proven else self.enter_speed
        if abs(observed)>threshold:self.reset('OBSERVED_MOVING');return False
        if self.confirmed and abs(observed)>self.enter_speed:
            if self.above_enter_since is None:self.above_enter_since=now
            if now-self.above_enter_since>=.15:self.reset('OBSERVED_MOVING');return False
        else:self.above_enter_since=None
        if self.anchor is not None:
            angle=math.atan2(math.sin(pose[2]-self.anchor[2]),math.cos(pose[2]-self.anchor[2]))
            if math.hypot(pose[0]-self.anchor[0],pose[1]-self.anchor[1])>self.position_span or abs(angle)>self.heading_span:
                self.reset('POSE_MOVING');return False
        if self.last_stamp is not None and stamp<self.last_stamp:self.reset('CLOCK_RESET');return False
        if self.samples and now<self.samples[-1][0]:self.reset('CLOCK_RESET');return False
        if stamp!=self.last_stamp:
            self.samples.append((now,stamp,tuple(pose)));self.last_stamp=stamp
        while len(self.samples)>2 and now-self.samples[1][0]>=self.window:self.samples.popleft()
        if not self.samples:return False
        anchor=self.samples[0][2][2]
        yaws=[math.atan2(math.sin(p[2][2]-anchor),math.cos(p[2][2]-anchor)) for p in self.samples]
        span=math.hypot(max(p[2][0] for p in self.samples)-min(p[2][0] for p in self.samples),max(p[2][1] for p in self.samples)-min(p[2][1] for p in self.samples))
        if span>self.position_span or max(yaws)-min(yaws)>self.heading_span:
            self.reset('POSE_MOVING');return False
        if len(self.samples)>=self.min_samples and self.samples[-1][0]-self.samples[0][0]>=self.window:
            self.confirmed=True;self.state='CONFIRMED_STATIONARY'
            self.motion_proven=False
            if self.anchor is None:self.anchor=tuple(pose)
        return self.confirmed
