"""Fixed-lag command-model observer; estimates are never stop evidence."""
from __future__ import division
import math
from collections import deque

def finite(x):return not (math.isnan(x) or math.isinf(x))

class SpeedObserver(object):
    def __init__(self,acceleration,braking,max_speed=.5,gain=.4,history=2.):
        if not all(finite(x) and x>0 for x in (acceleration,braking,max_speed,gain,history)) or gain>1:
            raise ValueError('invalid speed observer configuration')
        self.acceleration=acceleration;self.braking=braking;self.max_speed=max_speed
        self.gain=gain;self.horizon=history;self.samples=deque()
        self.velocity=None;self.last=None;self.target=0.;self.measurement_key=None
        self.state='WAIT_MEASUREMENT'

    def reset(self):
        self.velocity=None;self.last=None;self.target=0.;self.samples.clear();self.measurement_key=None
        self.state='WAIT_MEASUREMENT'

    def update(self,now,target,measured,measurement_time,key):
        if not all(finite(x) for x in (now,target,measured,measurement_time)) or abs(target)>self.max_speed or abs(measured)>self.max_speed:
            self.reset();self.state='INVALID';return None
        if self.last is not None and not 0<=now-self.last<=.35:
            self.reset();self.state='TIME_GAP';return None
        if measurement_time>now+1e-6 or now-measurement_time>self.horizon:
            self.reset();self.state='STALE_MEASUREMENT';return None
        if self.velocity is None:self.velocity=measured
        elif self.last is not None:
            dt=now-self.last
            # Cross-direction commands brake to zero before accelerating backwards.
            aim=0. if self.velocity*self.target<0 else self.target
            rate=self.braking if abs(aim)<abs(self.velocity) or aim*self.velocity<0 else self.acceleration
            self.velocity+=max(-rate*dt,min(rate*dt,aim-self.velocity))
        self.samples.append((now,self.velocity))
        while len(self.samples)>2 and now-self.samples[1][0]>self.horizon:self.samples.popleft()
        if key!=self.measurement_key:
            self.measurement_key=key;historical=None
            history=list(self.samples)
            for a,b in zip(history,history[1:]):
                if a[0]<=measurement_time<=b[0] and b[0]>a[0]:
                    u=(measurement_time-a[0])/(b[0]-a[0]);historical=a[1]+u*(b[1]-a[1]);break
            if historical is None:
                self.state='WARMUP'
            else:
                error=measured-historical
                # Correct the retained trajectory too, so old innovations are not applied twice.
                correction=self.gain*error
                self.samples=deque((t,max(-self.max_speed,min(self.max_speed,v+correction))) for t,v in self.samples)
                self.velocity=max(-self.max_speed,min(self.max_speed,self.velocity+correction))
                self.state='PREDICTED'
        self.last=now;self.target=target
        return self.velocity
