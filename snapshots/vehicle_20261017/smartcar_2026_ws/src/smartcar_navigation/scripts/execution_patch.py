"""Execution/recovery safeguards; never searches for or rewrites a global path."""
from __future__ import division
import math
from ackermann_core import Grid,front_position,advance,StopWindow,wrap
from inspection_depth import blocked,segment_blocked,coordinates,HALF
from nav_config import P

DEFAULTS=dict(enabled=True,depth_margin=.01,cusp_stop_distance=.05,
              cusp_braking=.15,cusp_delay=.55,depth_escape_limit=.08,
              depth_escape_speed=.04,geometry_retry_delay=2.,geometry_retry_limit=3,
              align_depth_with_channels=True)


def configure(values=None):
    values={} if values is None else values
    if not isinstance(values,dict) or set(values)-set(DEFAULTS):raise ValueError('Unknown execution_patch setting')
    result=dict(DEFAULTS);result.update(values)
    for key,value in result.items():
        if key in ('enabled','align_depth_with_channels'):
            if not isinstance(value,bool):raise ValueError(key+' must be boolean')
    # Explicit checks keep the same finite validation on Python 2.
    for key in set(result)-set(('enabled','align_depth_with_channels')):
        v=result[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or math.isnan(v) or math.isinf(v):
            raise ValueError('Nonfinite execution setting '+key)
    bounds=dict(depth_margin=(0.,.03),cusp_stop_distance=(.035,.08),cusp_braking=(.05,.5),
                cusp_delay=(.2,1.),depth_escape_limit=(.01,.10),depth_escape_speed=(.01,.05),
                geometry_retry_delay=(1.,10.),geometry_retry_limit=(1,5))
    for key,(low,high) in bounds.items():
        if not low<=result[key]<=high:raise ValueError('Out of bounds '+key)
    if int(result['geometry_retry_limit'])!=result['geometry_retry_limit']:raise ValueError('Noninteger retry limit')
    return result


def settings(navigator):
    return getattr(navigator,'execution_options',DEFAULTS)


def align_regions(regions,clearance):
    """Use measured 1/2,3/4,... inspection pairs, not an arbitrary map-grid yaw."""
    if not regions:return []
    if len(regions)!=10:raise ValueError('Ten inspection points required for channel axes')
    axes=[]
    for a,b in zip(regions[::2],regions[1::2]):
        dx,dy=b['x']-a['x'],b['y']-a['y']
        if not .2<=math.hypot(dx,dy)<=10.:raise ValueError('Invalid paired inspection points')
        axes.append(math.atan2(dy,dx))
    x=sum(math.cos(2*a) for a in axes);y=sum(math.sin(2*a) for a in axes)
    if math.hypot(x,y)<1.:raise ValueError('Inconsistent channel axes')
    axis=math.atan2(y,x)/2.
    if math.sin(axis)<0.:axis+=math.pi
    if any(abs(math.sin(a-axis))>math.sin(math.radians(10.)) for a in axes):
        raise ValueError('Paired inspection points disagree on channel direction')
    return [dict(r,axis_yaw=r.get('axis_yaw',axis),clearance=clearance) for r in regions]


def geometry_wait(navigator,reason,now):
    options=settings(navigator)
    navigator.retry_after=now+options['geometry_retry_delay']
    navigator.retry_geometry_failures=getattr(navigator,'retry_geometry_failures',0)+1
    exhausted=options['enabled'] and navigator.retry_geometry_failures>=options['geometry_retry_limit']
    if exhausted:
        hold_recovery(navigator,reason,now,getattr(navigator,'retry_last_pose',None),recheck=True)
    navigator.status(('%s: geometry %d/%d; original goal retained; '%
                      ('LASER_RECOVERY_HOLD' if exhausted else 'RECOVERY_GEOMETRY_WAIT',
                       navigator.retry_geometry_failures,options['geometry_retry_limit']))+reason)


class RecoveryGeometryError(RuntimeError):
    """Valid stationary sensor evidence, but no currently admissible geometry."""


def hold_recovery(navigator,reason,now,pose,recheck=False):
    """Escape budgets stay latched; a safe return to NORMAL may be rechecked."""
    navigator.state='LASER_RECOVERY_HOLD'
    navigator.laser_hold_check=None
    if pose is not None:
        navigator.laser_hold_check=dict(after=now+settings(navigator)['geometry_retry_delay'],
            origin=tuple(pose),window=StopWindow(),map_window=StopWindow(),scans=set(),
            normal_scans=set(),allow_raw=bool(recheck),reason=reason,last_report=0.)
    navigator.status('LASER_RECOVERY_HOLD: '+reason+(
        '; normal-clearance recheck; raw escape '+('eligible' if recheck else 'latched')
        if navigator.laser_hold_check else '; missing pose; manual reset required'))


def recheck_recovery_hold(navigator,pose,now):
    """Revalidate a stopped goal, never release a raw recovery motion budget."""
    record=getattr(navigator,'laser_hold_check',None)
    if record is None or now<record['after']:return
    origin=record['origin']
    if math.hypot(pose[0]-origin[0],pose[1]-origin[1])>.03 or abs(wrap(pose[2]-origin[2]))>math.radians(3.):
        # Invalidate the old escape geometry, then wait for a new stable normal
        # pose. This never authorizes escape from a still-blocked footprint.
        hold_recovery(navigator,'pose moved during recovery hold',now,pose)
        return
    try:
        navigator.sensors()
        speed=abs(navigator.actuator_state()[1])
        if speed>.005:raise RecoveryGeometryError('wheel speed %.4fm/s exceeds stop threshold'%speed)
        stopped=record['window'].update(now,navigator.stop_pose(pose))
        map_stopped=record['map_window'].update(now,pose)
        # Use the same mandatory footprint as ordinary execution. Fresh-frame
        # hysteresis supplies confidence without an extra permanent 1cm gate.
        normal_hits=navigator.grid.collision(pose,extra=P['safety_stop_extra'])
        stamp=getattr(navigator,'obstacle_stamp',None)
        if stamp is None:stamp=navigator.scan.header.stamp.to_nsec()
        if not normal_hits:
            record['normal_scans'].add(stamp)
            if len(record['normal_scans'])>5:record['normal_scans']=set(sorted(record['normal_scans'])[-5:])
            if stopped and len(record['normal_scans'])>=5:
                navigator.laser_hold_check=None
                navigator.wait_retry('normal clearance restored after recovery hold; stopped and five distinct scans; raw escape budget retained')
                return
            reason='normal clearance clear; stop=%s map_stable=%s scans=%d/5'%(stopped,map_stopped,len(record['normal_scans']))
        else:
            record['normal_scans']=set()
            reason='normal footprint blocked: '+','.join(sorted(normal_hits))
            if record['allow_raw']:
                raw=navigator.raw_recovery_grid(pose)
                if not raw.free(pose):raise RecoveryGeometryError('raw footprint blocked or unobserved')
                raw_stamp=getattr(raw,'scan_stamp',None)
                if raw_stamp is None:raw_stamp=navigator.scan.header.stamp.to_nsec()
                record['scans'].add(raw_stamp)
                if len(record['scans'])>5:record['scans']=set(sorted(record['scans'])[-5:])
                if stopped and map_stopped and len(record['scans'])>=5:
                    navigator.laser_hold_check=None
                    navigator.wait_retry('temporary raw hold cleared; full escape geometry and attempt limits must be rechecked')
                    return
                reason+='; raw clear, waiting for fresh scans and stable stop'
    except Exception as exc:
        record['window']=StopWindow();record['map_window']=StopWindow()
        record['scans']=set();record['normal_scans']=set()
        record['after']=now+settings(navigator)['geometry_retry_delay']
        reason=str(exc)
    if now-record['last_report']>=5.:
        record['last_report']=now
        navigator.status('RECOVERY_HOLD_WAIT: '+reason+'; raw_escape='+str(record['allow_raw']))


def cusp_speed_limit(remaining,braking,options):
    """Cap speed so reaction plus braking ends before the reserved cusp stop."""
    if not options['enabled']:return float('inf')
    if any(math.isnan(v) or math.isinf(v) for v in (remaining,braking)) or braking<=0:
        raise ValueError('Invalid cusp braking input')
    a=min(braking,options['cusp_braking']);delay=options['cusp_delay']
    # The zero cap is at the exact path cusp. Request the stopped CUSP state
    # earlier (at cusp_stop_distance), so the car can reach that finite gate.
    distance=max(0.,remaining)
    return max(0.,a*(math.sqrt(delay*delay+2*distance/a)-delay))


class ExecutionGrid(Grid):
    """Local-only view: body padding and depth padding are both explicit."""
    def __init__(self,base,depth_margin):
        self.__dict__.update(base.snapshot().__dict__)
        self.execution_depth_margin=depth_margin
    def collision(self,pose,detailed=False,extra=0.):
        hits=Grid.collision(self,pose,detailed,extra)
        rule=blocked(front_position(pose),self.depth_rules,extra+self.execution_depth_margin)
        if rule:
            hits=dict(hits);hits['INSPECTION_DEPTH_'+rule]=front_position(pose)
        return hits
    def arc(self,pose,distance,curvature):
        if self.depth_rules:
            n=max(1,int(math.ceil(abs(distance)/min(.01,self.res/4.))))
            angle=abs(distance*curvature/n)
            sagitta=(math.hypot(1./curvature,P['wheelbase'])*(1-math.cos(angle/2.)) if abs(curvature)>1e-9 else 0.)
            previous=front_position(pose)
            for i in range(1,n+1):
                current=front_position(advance(pose,distance*i/n,curvature))
                if segment_blocked(previous,current,self.depth_rules,self.execution_depth_margin+sagitta):return None
                previous=current
        return Grid.arc(self,pose,distance,curvature)


class DepthEscape(object):
    """Allow only bounded, monotonically decreasing pre-existing depth violations.

    Physical obstacles, external region and all previously valid depth rules stay
    hard. Moving sideways out of a forbidden strip cannot erase a violation.
    """
    def __init__(self,rules,origin,limit,padding=0.):
        self.rules=list(rules);self.padding=max([padding]+[r.get('clearance',0.) for r in rules]);self.budgets={}
        for rule in self.rules:
            if blocked(origin,[rule],self.padding):
                along,cross=coordinates(origin,rule)
                amount=rule['direction']*along-HALF+self.padding
                if amount>limit+1e-9:raise RecoveryGeometryError('depth violation exceeds bounded exit limit')
                self.budgets[rule['id']]=max(0.,amount)
        self.floor=dict(self.budgets)
    def amount(self,point,rule):
        return rule['direction']*coordinates(point,rule)[0]-HALF+self.padding
    def collision(self,point,extra=0.):
        for rule in self.rules:
            identity=rule['id']
            if identity not in self.budgets:
                if blocked(point,[rule],extra+self.padding):return True
            else:
                along,cross=coordinates(point,rule)
                if abs(cross)>HALF-self.padding:return True
                if self.amount(point,rule)>self.floor[identity]+1e-6:return True
        return False
    def monotonic(self,a,b):
        for rule in self.rules:
            if rule['id'] in self.budgets:
                if self.amount(b,rule)>self.amount(a,rule)+1e-8:return False
        return True
    def update_observed(self,point,previous=None):
        if previous:
            for key,value in previous.items():
                if key in self.floor:self.floor[key]=min(self.floor[key],value)
        if self.collision(point):raise RecoveryGeometryError('depth exit increased violation or left its channel')
        for rule in self.rules:
            if rule['id'] in self.floor:
                # Runtime feedback jitter receives 5 mm, never beyond the initial limit.
                self.floor[rule['id']]=min(self.floor[rule['id']],self.amount(point,rule)+.005)
        return dict(self.floor)
