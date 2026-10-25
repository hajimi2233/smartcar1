"""Map-bound corridor surface groups. Python 2/3, no ROS or TF publication."""
from __future__ import division
import json
import math
import numpy as np

DEFAULTS = dict(enabled=False, definition_file='', enter_inset=.04, exit_margin=.12,
                confirm_time=.3, normal_angle_deg=25., side_weight=.35,
                anchor_weight=.55, other_weight=.10, anchor_min_points=8,
                anchor_max_rms=.06, group_max_rms=.08)


def configure(values):
    if not isinstance(values, dict) or set(values)-set(DEFAULTS):
        raise ValueError('Invalid corridor_weighting keys')
    cfg=dict(DEFAULTS);cfg.update(values)
    if not isinstance(cfg['enabled'],bool):raise ValueError('enabled must be boolean')
    try:string_type=basestring
    except NameError:string_type=str
    if not isinstance(cfg['definition_file'],string_type):raise ValueError('definition_file must be text')
    for key in set(DEFAULTS)-set(('enabled','definition_file')):
        v=cfg[key]
        if isinstance(v,bool) or not isinstance(v,(int,float)) or not np.isfinite(v):
            raise ValueError('Invalid finite corridor setting '+key)
    if not 0<=cfg['enter_inset']<=.15 or not 0<=cfg['exit_margin']<=.25:
        raise ValueError('Invalid corridor hysteresis distances')
    if not .1<=cfg['confirm_time']<=1. or not 5<=cfg['normal_angle_deg']<=40:
        raise ValueError('Invalid corridor timing/normal angle')
    if any(not .01<=cfg[k]<=.9 for k in ('side_weight','anchor_weight','other_weight')):
        raise ValueError('Invalid positive group weights')
    if abs(sum(cfg[k] for k in ('side_weight','anchor_weight','other_weight'))-1.)>1e-8:
        raise ValueError('Corridor weights must sum to one')
    if int(cfg['anchor_min_points'])!=cfg['anchor_min_points'] or not 8<=cfg['anchor_min_points']<=100:
        raise ValueError('Invalid anchor_min_points')
    if not .02<=cfg['anchor_max_rms']<=cfg['group_max_rms']<=.10:
        raise ValueError('Invalid group quality limits')
    return cfg


class CorridorPolicy(object):
    def __init__(self, settings):
        self.cfg=configure(settings)
        self.definition=None;self.groups=None;self.map_key=None
        self.state='OUTSIDE';self.pending=None;self.pending_since=None;self.last_stamp=None
        self.reason='DISABLED' if not self.cfg['enabled'] else 'WAIT_MAP'

    def bind(self, map_key, samples, resolution, definition=None):
        self.groups=None;self.definition=None;self.map_key=map_key
        self.state='OUTSIDE';self.pending=None;self.last_stamp=None
        if not self.cfg['enabled']:self.reason='DISABLED';return
        if definition is None:
            try:
                with open(self.cfg['definition_file']) as f:definition=json.load(f)
            except (IOError,ValueError) as exc:
                self.reason='DEFINITION_UNAVAILABLE: '+str(exc);return
        if definition.get('schema_version')!=1 or definition.get('frame_id')!='map':
            raise ValueError('Invalid corridor definition schema/frame')
        if definition.get('map_sha256')!=map_key:
            self.reason='MAP_MISMATCH';return
        origin=np.asarray(definition.get('origin',[]),dtype=float)
        boxes=np.asarray(definition.get('corridors_grid_m',[]),dtype=float)
        extent=np.asarray(definition.get('internal_wall_y_range',[]),dtype=float)
        if (origin.shape!=(3,) or boxes.ndim!=2 or boxes.shape[1]!=4 or not len(boxes)
                or extent.shape!=(2,) or not all(np.all(np.isfinite(v)) for v in (origin,boxes,extent))
                or np.any(boxes[:,1]-boxes[:,0]<.5) or np.any(boxes[:,3]-boxes[:,2]<.5)
                or extent[1]-extent[0]<.5):
            raise ValueError('Invalid corridor map geometry')
        samples=np.asarray(samples,dtype=float)
        if samples.ndim!=2 or samples.shape[1]!=4 or not np.all(np.isfinite(samples)):
            raise ValueError('Invalid map surface samples')
        self.definition=definition;self.origin=origin;self.boxes=boxes
        angle=origin[2];self.axis=np.array([-math.sin(angle),math.cos(angle)])
        local=self.grid_coordinates(samples[:,:2])
        cosine=math.cos(math.radians(self.cfg['normal_angle_deg']))
        longitudinal=np.abs(np.dot(samples[:,2:],self.axis))>=cosine
        lateral=np.abs(np.dot(samples[:,2:],np.array([self.axis[1],-self.axis[0]])))>=cosine
        # Only actual matched returns can contribute. These masks merely label
        # existing map surfaces; they cannot create rays through occluding walls.
        anchor=longitudinal & ((local[:,1]<=extent[0]-.25)|(local[:,1]>=extent[1]+.25))
        side=lateral & ~anchor
        other=~(anchor|side)
        self.groups=[]
        for number,role,mask in ((101,'side',side),(102,'anchor',anchor),(103,'other',other)):
            if int(mask.sum())<3:continue
            self.groups.append(dict(id=number,role=role,samples=samples[mask].tolist(),resolution=resolution,
                                    weight=self.cfg[role+'_weight'],
                                    min_points=int(self.cfg['anchor_min_points']) if role=='anchor' else 8))
        if not any(g['role']=='anchor' for g in self.groups):
            self.groups=None;self.reason='NO_ANCHOR_MAP_SURFACES';return
        self.reason='READY'

    def grid_coordinates(self, points):
        delta=np.asarray(points)-self.origin[:2];c,s=math.cos(self.origin[2]),math.sin(self.origin[2])
        return np.dot(delta,np.array([[c,-s],[s,c]]))

    def select(self, pose, stamp):
        if self.groups is None:return None
        if not np.all(np.isfinite(pose)) or not np.isfinite(stamp):raise ValueError('Invalid corridor pose/time')
        if self.last_stamp is not None and stamp<self.last_stamp:
            self.state='OUTSIDE';self.pending=None
        self.last_stamp=stamp
        x,y=self.grid_coordinates(np.asarray(pose[:2]).reshape(1,2))[0]
        margin=-self.cfg['enter_inset'] if self.state=='OUTSIDE' else self.cfg['exit_margin']
        inside=any(a-margin<=x<=b+margin and c-margin<=y<=d+margin for a,b,c,d in self.boxes)
        proposed='INSIDE' if inside else 'OUTSIDE'
        if proposed==self.state:self.pending=None
        elif proposed!=self.pending:self.pending=proposed;self.pending_since=stamp
        elif stamp-self.pending_since>=self.cfg['confirm_time']:
            self.state=proposed;self.pending=None
        return self.groups if self.state=='INSIDE' else None

    def context(self):
        return dict(corridor_state=self.state,corridor_policy=self.reason,
                    corridor_weighting_active=self.groups is not None and self.state=='INSIDE',
                    group_rejection_enabled=False)

    def quality(self, fitted, detail, active):
        detail.update(self.context())
        if not active:return fitted,detail
        stats=detail.get('surface_groups',[])
        anchor=next((g for g in stats if g['role']=='anchor'),None)
        reliable=(anchor is not None and anchor['points']>=self.cfg['anchor_min_points']
                  and anchor['rms']<=self.cfg['anchor_max_rms'])
        detail['longitudinal_reliable']=bool(reliable)
        detail['anchor_points']=0 if anchor is None else anchor['points']
        detail['anchor_rms']=None if anchor is None else anchor['rms']
        detail['other_group_warning']=any(g['role']=='other' and g['rms']>self.cfg['group_max_rms'] for g in stats)
        if fitted is None:
            detail['longitudinal_reliable']=False
            return fitted,detail
        # Map/field differences are diagnostic, never a second whole-pose veto.
        # match() has already applied the original fit/support/correction checks.
        detail['group_residual_warning']=any(
            g['role'] in ('side','anchor') and g['rms']>self.cfg['group_max_rms'] for g in stats)
        if detail['reason']=='MATCHED' and not reliable:
            detail['reason']='PARTIAL'
        return fitted,detail
