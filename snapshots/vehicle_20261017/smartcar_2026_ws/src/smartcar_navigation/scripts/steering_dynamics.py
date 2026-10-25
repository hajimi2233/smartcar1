"""Measured equivalent-angle response, with command history and Py2 support."""
import math, os, time

response_clock=getattr(time,'monotonic',lambda:os.times()[4])

class ResponseModel(object):
    def __init__(self,settings):
        required=('left_delay','right_delay','left_small_rate','right_small_rate',
                  'left_large_rate','right_large_rate','small_amplitude_deg','large_amplitude_deg')
        if settings.get('version')!=1 or any(k not in settings for k in required):
            raise ValueError('incomplete measured steering response')
        self.settings={k:float(settings[k]) for k in required}
        for side in ('left','right'):
            self.settings[side+'_small_delay']=float(settings.get(side+'_small_delay',self.settings[side+'_delay']))
        if any(math.isnan(v) or math.isinf(v) or v<=0 for v in self.settings.values()):
            raise ValueError('invalid measured steering response')
        if not 0<self.settings['small_amplitude_deg']<self.settings['large_amplitude_deg']<=90:
            raise ValueError('invalid steering amplitude bands')
        if max(self.settings[k] for k in self.settings if k.endswith('_delay'))>1:
            raise ValueError('steering delay exceeds one second')
        if any(self.settings[k]>3 for k in required if k.endswith('_rate')):
            raise ValueError('steering rate exceeds measured model range')
    def timing(self,change):
        side='left' if change>0 else 'right';s=self.settings
        weight=max(0.,min(1.,(math.degrees(abs(change))-s['small_amplitude_deg'])/
                          (s['large_amplitude_deg']-s['small_amplitude_deg'])))
        delay=s[side+'_small_delay']+weight*(s[side+'_delay']-s[side+'_small_delay'])
        return delay,s[side+'_small_rate']+weight*(s[side+'_large_rate']-s[side+'_small_rate'])

class ResponseState(object):
    def __init__(self,model,now,angle,active=None,pending=None,last_target=None,preview=None):
        self.model=model;self.now=now;self.angle=angle;self.active=active
        self.pending=list(pending or []);self.last_target=last_target
        self.preview=preview
    def clone(self):
        return ResponseState(self.model,self.now,self.angle,self.active,self.pending,self.last_target,self.preview)
    def request(self,target):
        if self.last_target is not None and abs(target-self.last_target)<1e-5:return
        self.last_target=target
        if abs(target-self.angle)<1e-5 and not self.pending:
            self.active=(target,self.model.timing(1e-6)[1]);return
        delay,rate=self.model.timing(target-self.angle)
        due=max(self.now+delay,self.pending[-1][0] if self.pending else self.now)
        self.pending.append((due,target,rate))
    def advance(self,target,dt):
        self.request(target);end=self.now+dt
        while self.now<end-1e-10:
            while self.pending and self.pending[0][0]<=self.now+1e-10:
                unused,target,rate=self.pending.pop(0);self.active=(target,rate)
            finish=min(end,self.pending[0][0]) if self.pending else end
            if self.active is not None:
                target,rate=self.active;change=rate*(finish-self.now)
                self.angle+=max(-change,min(change,target-self.angle))
            self.now=finish
        return self.angle
    def completion_time(self,target):
        delay,rate=self.model.timing(target-self.angle)
        if self.last_target is not None and abs(target-self.last_target)<.002:
            if self.pending:delay=max(0.,self.pending[-1][0]-self.now)
            elif self.active is not None and abs(target-self.active[0])<.002:delay=0.
        return delay+abs(target-self.angle)/rate
    def bound_angles(self):
        return [self.angle]+([self.active[0]] if self.active else [])+[p[1] for p in self.pending]

class SteeringHistory(object):
    def __init__(self,settings):
        self.model=ResponseModel(settings);self.reset()
    def reset(self):self.pending=[];self.active=None;self.last_target=None;self.last_time=None;self.preview=None
    def snapshot(self,now,actual):
        if self.last_time is not None and now<self.last_time:self.reset()
        self.last_time=now
        while self.pending and self.pending[0][0]<=now:
            unused,target,rate=self.pending.pop(0);self.active=(target,rate)
        return ResponseState(self.model,now,actual,self.active,self.pending,self.last_target,self.preview)
    def commit(self,now,target,actual):
        state=self.snapshot(now,actual);state.request(target)
        self.pending=state.pending;self.active=state.active;self.last_target=state.last_target
        if len(self.pending)>100:raise RuntimeError('steering command backlog exceeds model horizon')
