#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Two-click wall annotations, explicit commit, map-bound atomic persistence."""
import copy
import json
import threading
import math
import rospy
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from wall_features import GROUPS, fingerprint, save_file, validate_groups
from map_wall_extraction import extract_groups, map_geometry, extract_wall


class WallEditor(object):
    def __init__(self):
        self.lock=threading.RLock()
        self.path=rospy.get_param('~file')
        self.frame=rospy.get_param('~map_frame','map')
        self.map_msg=None;self.geometry=None
        self.radius=float(rospy.get_param('~selection_radius',.30))
        self.preview=dict(inside=[],outside=[]);self.preview_errors=[]
        self.saved_selectors=dict(inside=[],outside=[])
        self.map_id=None; self.group=None; self.pending=None
        self.active=dict(inside=[],outside=[]);self.draft=copy.deepcopy(self.active)
        self.definition=rospy.Publisher('/wall_features/definition',String,queue_size=1,latch=True)
        self.markers=rospy.Publisher('/wall_features/markers',MarkerArray,queue_size=1,latch=True)
        self.status=rospy.Publisher('/wall_features/editor_status',String,queue_size=10,latch=True)
        self.reply=rospy.Publisher('/wall_features/reply',String,queue_size=10)
        rospy.Subscriber('/map',OccupancyGrid,self.on_map,queue_size=1)
        rospy.Subscriber('/wall_features/point',PointStamped,self.click,queue_size=20)
        rospy.Subscriber('/wall_features/command',String,self.command,queue_size=10)
        self.publish()

    def report(self,message):
        self.status.publish(message);rospy.loginfo(message)

    def on_map(self,msg):
        with self.lock:
            key=fingerprint(msg)
            if key==self.map_id: return
            self.preview=dict(inside=[],outside=[]);self.preview_errors=[]
            self.map_msg=msg;self.geometry=map_geometry(msg)
            self.map_id=key;self.group=None;self.pending=None
            self.active=dict(inside=[],outside=[])
            self.saved_selectors=dict(inside=[],outside=[])
            self.draft=dict(inside=[],outside=[])
            try:
                if msg.header.frame_id.lstrip('/')!=self.frame: raise ValueError('Map frame mismatch')
                self.load_selection()
                self.draft=copy.deepcopy(self.saved_selectors)
                self.report('Wall selections loaded; targets extracted from current map')
            except Exception as exc:
                self.report('No valid saved walls: '+str(exc))
            self.publish()

    def build_preview(self):
        self.preview=dict(inside=[],outside=[]);self.preview_errors=[]
        if self.map_msg is None: raise ValueError('Wait for map first')
        geometry=self.geometry
        for group in GROUPS:
            for wall in self.draft[group]:
                try: self.preview[group].append(extract_wall(geometry,wall,self.radius))
                except ValueError as exc: self.preview_errors.append('%s #%d: %s'%(group,wall['id'],exc))
        return self.preview

    def load_selection(self):
        with open(self.path) as stream: data=json.load(stream)
        if data.get('map_sha256')!=self.map_id or data.get('frame_id')!=self.frame:
            raise ValueError('Wall file belongs to another map/frame')
        if data.get('schema_version') not in (1,2,3): raise ValueError('Invalid wall schema')
        selectors=validate_groups(data['selectors'] if data['schema_version'] in (2,3) else data['groups'])
        for group in GROUPS:
            for wall in selectors[group]: wall.setdefault('radius',self.radius)
        # Legacy hand-drawn targets are selections only; never activate them directly.
        try: targets=extract_groups(self.map_msg,selectors,self.radius)
        except ValueError:
            # Preserve legacy selections as editable draft even when extraction fails.
            self.draft=copy.deepcopy(selectors)
            raise
        self.active=targets;self.saved_selectors=selectors

    @staticmethod
    def summary(groups):
        return {group:[dict(id=w['id'],radius=w['radius'],occupied_cells=len(w['cells']),
                            surface_samples=len(w['samples'])) for w in groups[group]] for group in GROUPS}

    def execute(self,args):
        if not args: raise ValueError('Specify inside|outside|undo|delete GROUP ID|clear GROUP|radius METRES|preview|save|load|cancel|list')
        action=args[0]
        if action in GROUPS and len(args)==1:
            self.group=action;self.pending=None
        elif action=='radius' and len(args)==2:
            radius=float(args[1])
            if math.isnan(radius) or math.isinf(radius) or radius<=0:
                raise ValueError('Radius must be a positive finite number in metres')
            self.radius=radius
            for group in GROUPS:
                for wall in self.draft[group]: wall['radius']=radius
        elif action=='undo' and len(args)==1:
            if self.pending is not None: self.pending=None
            elif self.group and self.draft[self.group]: self.draft[self.group].pop()
        elif action=='delete' and len(args)==3 and args[1] in GROUPS:
            group,ident=args[1],int(args[2]);before=len(self.draft[group])
            self.draft[group]=[w for w in self.draft[group] if w['id']!=ident]
            if len(self.draft[group])==before: raise ValueError('No such wall ID')
        elif action=='clear' and len(args)==2 and args[1] in GROUPS+('all',):
            for group in GROUPS if args[1]=='all' else (args[1],): self.draft[group]=[]
            self.pending=None
        elif action=='cancel' and len(args)==1:
            self.draft=copy.deepcopy(self.saved_selectors);self.pending=None;self.group=None
        elif action=='preview' and len(args)==1:
            self.build_preview();self.publish()
            if self.preview_errors: raise ValueError('; '.join(self.preview_errors))
        elif action=='save' and len(args)==1:
            if not self.map_id: raise ValueError('Wait for map first')
            if self.pending is not None: raise ValueError('Finish the second endpoint or undo it before saving')
            targets=self.build_preview();self.publish()
            if self.preview_errors: raise ValueError('; '.join(self.preview_errors))
            save_file(self.path,targets,self.map_id,self.frame,selectors=self.draft)
            self.active=targets;self.saved_selectors=copy.deepcopy(self.draft)
            self.preview=dict(inside=[],outside=[])
        elif action=='load' and len(args)==1:
            if not self.map_id: raise ValueError('Wait for map first')
            self.load_selection()
            self.draft=copy.deepcopy(self.saved_selectors);self.pending=None
        elif action=='list' and len(args)==1:
            return json.dumps(dict(selected_group=self.group,radius=self.radius,draft=self.draft,active=self.summary(self.active),preview=self.summary(self.preview),errors=self.preview_errors),sort_keys=True)
        else: raise ValueError('Invalid wall command')
        if action not in ('save','preview'):
            self.preview=dict(inside=[],outside=[]);self.preview_errors=[]
        self.publish()
        return '%s; drawing=%s; draft inside=%d outside=%d; changes apply only after save' % (
            ' '.join(args),self.group,len(self.draft['inside']),len(self.draft['outside']))

    def command(self,msg):
        request={}
        with self.lock:
            try:
                request=json.loads(msg.data)
                message=self.execute(request['args']);ok=True
            except Exception as exc:
                message=str(exc);ok=False
                self.publish()
            self.report(message)
            self.reply.publish(json.dumps(dict(request_id=request.get('request_id'),ok=ok,message=message)))

    def click(self,msg):
        with self.lock:
            if not self.map_id or self.group is None:
                self.report('Wait for map, then run: walls inside OR walls outside');return
            if msg.header.frame_id.lstrip('/')!=self.frame:
                self.report('Set RViz Fixed Frame to '+self.frame);return
            p=[msg.point.x,msg.point.y]
            import math
            if any(math.isnan(v) or math.isinf(v) for v in p): return
            if self.pending is None: self.pending=p
            else:
                group=self.draft[self.group]
                wall=dict(id=max([w['id'] for w in group]+[0])+1,start=self.pending,end=p,radius=self.radius)
                candidate=copy.deepcopy(self.draft);candidate[self.group].append(wall)
                try: validate_groups(candidate)
                except ValueError as exc: self.report(str(exc));return
                self.draft=candidate;self.pending=None
                self.preview=dict(inside=[],outside=[]);self.preview_errors=[]
                self.report('Draft %s wall #%d added; save to apply' % (self.group,wall['id']))
            self.publish()

    def publish(self):
        self.definition.publish(json.dumps(dict(schema_version=3,source='map_mask',frame_id=self.frame,map_sha256=self.map_id,groups=self.active)))
        clear=Marker();clear.action=Marker.DELETEALL
        marks=[clear]
        for group in GROUPS:
            for wall in self.draft[group]:
                m=Marker();m.header.frame_id=self.frame;m.ns='walls_'+group;m.id=wall['id']*2
                m.type=Marker.LINE_LIST;m.pose.orientation.w=1.;m.scale.x=.015
                m.color.r,m.color.g,m.color.b=(1.,.3,0.) if group=='inside' else (0.,.65,1.)
                m.color.a=1. if wall in self.saved_selectors[group] else .45
                m.points=[Point(p[0],p[1],.06) for p in (wall['start'],wall['end'])]
                marks.append(m)
                label=copy.deepcopy(m);label.id+=1;label.type=Marker.TEXT_VIEW_FACING
                label.points=[];label.scale.z=.16
                label.pose.position=Point((wall['start'][0]+wall['end'][0])/2.,(wall['start'][1]+wall['end'][1])/2.,.25)
                label.text='select %s #%d%s'%(group,wall['id'],'' if wall in self.saved_selectors[group] else ' (draft)')
                marks.append(label)
        # Capsule outlines show the selection area, including round end caps.
        for group in GROUPS:
            for wall in self.draft[group]:
                a,b=wall['start'],wall['end'];radius=wall.get('radius',self.radius)
                yaw=math.atan2(b[1]-a[1],b[0]-a[0])
                outline=[]
                for center,angle in ((b,yaw-math.pi/2),(a,yaw+math.pi/2)):
                    for step in range(17):
                        theta=angle+step*math.pi/16.
                        outline.append(Point(center[0]+radius*math.cos(theta),center[1]+radius*math.sin(theta),.045))
                m=Marker();m.header.frame_id=self.frame;m.ns='selection_area_'+group;m.id=wall['id']
                m.type=Marker.LINE_STRIP;m.pose.orientation.w=1.;m.scale.x=.01;m.color.a=.6
                m.color.r,m.color.g,m.color.b=(1.,.3,0.) if group=='inside' else (0.,.65,1.)
                m.points=outline+outline[:1];marks.append(m)
        for name,groups,z in (('active',self.active,.07),('preview',self.preview,.10)):
            for group in GROUPS:
                for wall in groups[group]:
                    m=Marker();m.header.frame_id=self.frame;m.ns=name+'_map_cells_'+group;m.id=wall['id']
                    m.type=Marker.POINTS;m.pose.orientation.w=1.
                    m.scale.x=m.scale.y=wall['resolution'];m.color.a=.8
                    m.color.r,m.color.g,m.color.b=((.3,1.,.3) if name=='preview' else
                                                  ((1.,.3,0.) if group=='inside' else (0.,.65,1.)))
                    m.points=[Point(p[0],p[1],z) for p in wall['cells']];marks.append(m)
        if self.pending is not None:
            m=Marker();m.header.frame_id=self.frame;m.ns='pending_wall';m.id=0;m.type=Marker.SPHERE
            m.pose.orientation.w=1.;m.pose.position=Point(self.pending[0],self.pending[1],.1)
            m.scale.x=m.scale.y=m.scale.z=.10;m.color.r=m.color.g=m.color.a=1.;marks.append(m)
        self.markers.publish(MarkerArray(marks))


if __name__=='__main__':
    rospy.init_node('wall_editor');WallEditor();rospy.spin()
