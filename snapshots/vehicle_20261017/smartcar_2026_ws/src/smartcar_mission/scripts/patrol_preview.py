"""Mission metadata and explicitly simulated vision labels. Python 2/3, no ROS."""
from __future__ import division
import math,random,uuid

KINDS={'1':'FIRE','2':'WATER','3':'PEDESTRIAN','4':'ROAD'}
CONDITIONS={'A':'NORMAL','B':'ABNORMAL'}
LATERAL=('LATERAL','LATERAL_TURN_180')

def finite(value):
    return not isinstance(value,bool) and isinstance(value,(int,float)) and not math.isnan(value) and not math.isinf(value)

def report_value(target,report):
    if not isinstance(report,dict):raise ValueError('Missing patrol description')
    number=report.get('channel_number')
    if isinstance(number,bool) or number not in range(1,11) or target!='inspect_%d'%number:
        raise ValueError('Invalid patrol target number')
    if report.get('object_kind') not in KINDS or report.get('condition') not in CONDITIONS:
        raise ValueError('Invalid patrol category/condition')
    return dict(target_id=target,channel_number=number,object_kind=report['object_kind'],condition=report['condition'])


class PatrolPreview(object):
    """Only an acknowledged, correlated exit event can release a lateral hold."""
    def __init__(self,reports):
        self.reports={key:report_value(key,value) for key,value in reports.items()}
        self.run_id=None;self.sequence=0;self.active=False;self.display=None
        self.pending=None;self.token=None;self.mode=None;self.phase='IDLE';self.hold_for_exit=False
    def begin(self):
        self.run_id=uuid.uuid4().hex;self.sequence=0;self.active=True
        self.display=self.pending=self.token=self.mode=None;self.phase='STARTING';self.hold_for_exit=False
    def goal(self,label):
        if not self.active:raise ValueError('Preview run is inactive')
        self.sequence+=1;self.token=self.run_id+':'+str(self.sequence)
        self.pending=label if label in self.reports else None
        self.mode=None;self.hold_for_exit=False
        if self.pending is not None and self.display is None:
            self.display=self.pending;self.pending=None;self.phase='APPROACHING'
        elif self.pending is not None and self.pending!=self.display:
            self.phase='WAIT_NAV_MODE'
        else:self.pending=None;self.phase='TRANSIT'
        return self.token
    def stage(self,event):
        if (not self.active or not isinstance(event,dict) or event.get('schema_version')!=1
                or event.get('goal_token')!=self.token):return False
        text=event.get('status','')
        if not isinstance(text,(str,type(u''))):return False
        if text.startswith('GOAL_MODE:'):
            self.mode=event.get('maneuver_mode')
            if self.pending is not None:
                if self.mode in LATERAL or self.hold_for_exit:
                    self.hold_for_exit=True;self.phase='EXIT_WAIT'
                elif self.mode in ('STRAIGHT','NORMAL','TURN_90_LEFT','TURN_90_RIGHT','POSITION_ONLY'):
                    self.display=self.pending;self.pending=None;self.phase='APPROACHING'
                # Unknown modes cannot prematurely attribute the old channel.
            return True
        if (self.pending is not None and self.hold_for_exit
                and event.get('maneuver_mode') in LATERAL and event.get('line_stage')=='RETREAT_LINE'
                and text.startswith('LINE_CONFIRMED:') and 'planning line travel only' in text):
            self.display=self.pending;self.pending=None;self.phase='APPROACHING';self.hold_for_exit=False
            return True
        return False
    def arrived(self,label):
        # If a route has no explicit first exit stage, do not guess its timing.
        # Arrival is the last safe attribution boundary available to this UI.
        if self.active and label in self.reports:
            self.display=label;self.pending=None;self.phase='ARRIVED';self.hold_for_exit=False
    def clear(self,phase='IDLE'):
        self.active=False;self.display=self.pending=self.token=self.mode=None;self.phase=phase;self.hold_for_exit=False
    def snapshot(self,stamp):
        if not finite(stamp):raise ValueError('Invalid metadata time')
        return dict(schema_version=1,source='patrol_plan',run_id=self.run_id,active=self.active,
            goal_token=self.token,phase=self.phase,pending_target_id=self.pending,
            display= None if self.display is None else dict(self.reports[self.display]),stamp=stamp)


class SimulatedLabels(object):
    """20% per time window, never per video frame; synthetic output stays marked."""
    def __init__(self,error_probability=.20,period=1.,ttl=3.,rng=None):
        if not finite(error_probability) or not 0<=error_probability<=1:raise ValueError('Error probability must be 0..1')
        if not finite(period) or not .2<=period<=10:raise ValueError('Update period must be .2..10 seconds')
        if not finite(ttl) or not 1<=ttl<=10:raise ValueError('Context TTL must be 1..10 seconds')
        self.probability=error_probability;self.period=period;self.ttl=ttl
        self.rng=rng or random.Random();self.context=None;self.received=None
        self.identity=None;self.next_sample=None;self.last=None;self.reason='WAIT_PATROL'
    def update(self,data,now):
        if not finite(now):raise ValueError('Invalid receipt time')
        if (not isinstance(data,dict) or data.get('schema_version')!=1 or data.get('source')!='patrol_plan'
                or not isinstance(data.get('active'),bool)):raise ValueError('Invalid patrol context')
        if data['active']:
            if not isinstance(data.get('run_id'),(str,type(u''))) or not data['run_id']:raise ValueError('Missing run id')
            if data.get('display') is not None:
                report_value(data['display'].get('target_id'),data['display'])
        self.context=data;self.received=now;self.reason='READY' if data['active'] else 'PATROL_INACTIVE'
        if not data['active']:self.identity=self.next_sample=self.last=None
    def invalidate(self):
        self.context=None;self.received=None;self.identity=self.next_sample=self.last=None;self.reason='INVALID_CONTEXT'
    def sample(self,now):
        if not finite(now):raise ValueError('Invalid sample time')
        if self.received is None:return None
        if now<self.received or now-self.received>self.ttl:
            self.reason='CONTEXT_STALE';self.identity=self.next_sample=self.last=None;return None
        context=self.context
        if not context['active'] or context.get('display') is None:return None
        report=context['display'];target=report['target_id'];identity=(context['run_id'],target)
        if identity!=self.identity or self.next_sample is None or now>=self.next_sample:
            kind=KINDS[report['object_kind']];status=CONDITIONS[report['condition']]
            injected=self.rng.random()<self.probability
            if injected:
                # An error always picks a different object category.
                kind=self.rng.choice(sorted(k for k in KINDS.values() if k!=kind))
                status=self.rng.choice(('NORMAL','ABNORMAL'))
            self.identity=identity;self.next_sample=now+self.period
            self.last=dict(schema_version=1,source='SIMULATED_PATROL_LABEL',simulated=True,
                run_id=context['run_id'],target_id=target,channel_number=report['channel_number'],
                category=kind,status=status,label=status+'_'+kind,is_abnormal=status=='ABNORMAL',
                injected_error=injected,error_probability=self.probability,sample_time=now)
        result=dict(self.last);result['phase']=context.get('phase');result['pending_target_id']=context.get('pending_target_id')
        return result
