#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import json,math,os,sys,subprocess
sys.path.insert(0,'/home/smartcar/smartcar_2026_ws/src/smartcar_mission/scripts')
sys.path.insert(0,'/home/smartcar/smartcar_2026_ws/src/smartcar_navigation/scripts')
from inspection_depth import validate_regions
from std_msgs.msg import String
import rospy,rosnode
from multi_goal_nav import MultiGoalNav
from std_srvs.srv import TriggerResponse
from nav_msgs.msg import OccupancyGrid
from low_cost_lines import map_key
from inspection_reporting import parse_description,cached_audio

def read_route(path):
    with open(path) as f:data=json.load(f)
    if data['frame_id']!='map' or data['target_reference']!='front_axle_midpoint':raise ValueError('Wrong frame/reference')
    layout,reports=parse_description(data['description'],count=9)
    if data['layout']!=layout or data['reports']!=reports:raise ValueError('Description/report mismatch')
    expected=['inspect_%d'%i for i in range(1,10)]
    visits=[]
    for q in data['steps']:
        validate_regions(q['regions'])
        if any(math.isnan(q[k]) or math.isinf(q[k]) for k in ('x','y','yaw')):raise ValueError('Nonfinite pose')
        if q['target_id']=='inspect_10':raise ValueError('Excluded target')
        if q['visit'] is not None:
            visits.append(q['target_id']);r=q['regions'][int(q['target_id'].split('_')[1])-1]
            if r['type']!=q['inspection_type'] or math.hypot(r['x']-q['x'],r['y']-q['y'])>1e-8:raise ValueError('Mismatched inspection metadata')
            if q['label']!=q['target_id'] or q['report']!=reports[q['target_id']] or q['inspection_type']!=reports[q['target_id']]['condition']:raise ValueError('Speech is not bound to actual channel')
    if sorted(visits,key=lambda name:int(name.split('_')[1]))!=expected or data['steps'][-1]['label']!='END':raise ValueError('Must visit channels 1..9 once and return to end')
    return data

class RequestedPatrol(MultiGoalNav):
    def __init__(self,data):
        rospy.set_param('~plan_file','')
        rospy.set_param('~speech_enabled',True)
        MultiGoalNav.__init__(self)
        self.steps=data['steps'];self.points=[(q['x'],q['y'],q['yaw']) for q in self.steps];self.labels=[q['label'] for q in self.steps]
        self.reports=dict(data['reports'])
        self.speaker.audio=cached_audio('/home/smartcar/smartcar_2026_ws')
        missing=set(q['speech_text'] for q in self.reports.values())-set(self.speaker.audio)
        if missing:raise ValueError('Required speech cache missing: '+repr(sorted(missing)))
        self.publish_marks();self.status('READY: 9 channels with arrival speech; order 1 2 4 6 8 7 5 3 9 then external return and END; call execute to start')
    def on_goal_click(self,msg):
        self.status('REJECTED: fixed requested route; clicks do not change this queue')
    def undo(self,request):
        return TriggerResponse(False,'Fixed requested route; reload instead of undo')
    def execute(self,request):
        if self.inspection_goal_pub.get_num_connections()<1:return TriggerResponse(False,'Navigation inspection-goal subscriber is not ready')
        return MultiGoalNav.execute(self,request)
    def publish_current(self):
        self.publish_marks();q=self.steps[self.index]
        self.inspection_goal_pub.publish(String(data=json.dumps(dict(frame_id='map',x=q['x'],y=q['y'],yaw=q['yaw'],target_id=q['target_id'],regions=q['regions']))))
        self.status('GOAL %d/%d: %s'%(self.index+1,len(self.points),q['label']))

def main():
    path=os.path.join(os.path.dirname(os.path.abspath(__file__)),'requested_route.json');data=read_route(path)
    if '--speech-test' in sys.argv:
        audio=cached_audio('/home/smartcar/smartcar_2026_ws')
        for number in range(1,10):
            text=data['reports']['inspect_%d'%number]['speech_text']
            print(text.encode('utf-8') if sys.version_info[0]<3 else text)
            sys.stdout.flush()
            subprocess.check_call(['paplay',audio[text][0]])
        return
    if '--preview' in sys.argv:
        for q in data['steps']:
            line='%s x=%.4f y=%.4f heading=%.1fdeg'%(q['label'],q['x'],q['y'],math.degrees(q['yaw']))
            if q['target_id'] in data['reports']:line+=u' speech='+data['reports'][q['target_id']]['speech_text']
            print(line.encode('utf-8') if sys.version_info[0]<3 else line)
        print('Preview only; channels 1..9 once, external return, then END. No drive goal sent.');return
    if '/multi_goal_nav' in rosnode.get_node_names():raise RuntimeError('An existing queue is loaded. Exit its terminal before loading this route.')
    rospy.init_node('multi_goal_nav')
    with open(os.path.join(os.path.dirname(path),'low_cost_lines.json')) as f:allowed=json.load(f)['maps']
    current=rospy.wait_for_message('/map',OccupancyGrid,timeout=5.)
    if map_key(current) not in allowed:raise ValueError('Current map differs from map_20261012_192707; queue not loaded')
    RequestedPatrol(data);rospy.spin()
if __name__=='__main__':main()
