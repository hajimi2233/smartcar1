#!/usr/bin/env python
from __future__ import print_function
import os, csv, json, subprocess, threading
import rospy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class Preview(object):
    def __init__(self):
        self.lock=threading.Lock()
        self.info=rospy.get_param('~map_info');self.points=rospy.get_param('~points')
        self.output=rospy.get_param('~output');self.layout=rospy.get_param('~layout')
        if not os.path.isdir(self.output): os.makedirs(self.output)
        self.pub=rospy.Publisher('/inspection/preview',MarkerArray,queue_size=1,latch=True)
        self.status=rospy.Publisher('/inspection/preview_status',String,queue_size=1,latch=True)
        self.sub=rospy.Subscriber('/inspection/layout',String,self.callback,queue_size=1)
        self.callback(String(self.layout))
    def callback(self,msg):
        with self.lock:
            try:
                p=subprocess.Popen(['rosrun','smartcar_inspection','inspection_planner',msg.data,self.info,self.points],cwd=self.output,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
                out=p.communicate()[0]
                with open(os.path.join(self.output,'planner.log'),'wb') as f:f.write(out)
                if p.returncode: raise RuntimeError(out.decode('utf-8','replace'))
                with open(os.path.join(self.output,'motion_commands.jsonl')) as f: commands=[json.loads(line) for line in f if line.strip()]
                with open(self.points) as f: points=list(csv.DictReader(f))
                frame=commands[0]['frame_id'];markers=MarkerArray()
                clear=Marker();clear.action=Marker.DELETEALL;markers.markers.append(clear)
                for i,row in enumerate(points):
                    m=Marker();m.header.frame_id=frame;m.ns='named_points';m.id=i;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
                    m.pose.position.x=float(row['x']);m.pose.position.y=float(row['y']);m.pose.position.z=.6
                    m.pose.orientation.w=1;m.scale.z=.14;m.color.r=1;m.color.g=.9;m.color.a=1;m.text=row['point_id']
                    markers.markers.append(m)
                for i,c in enumerate(commands):
                    m=Marker();m.header.frame_id=frame;m.ns='command_order';m.id=i;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
                    m.pose.position.x=c['x']+.12;m.pose.position.y=c['y'];m.pose.position.z=.85+.09*(i%4)
                    m.pose.orientation.w=1;m.scale.z=.10;m.color.g=1;m.color.b=1;m.color.a=1
                    m.text='%d %s'%(c['task_id'],c['phase']);markers.markers.append(m)
                self.pub.publish(markers)
                self.status.publish('PLAN ONLY: %d commands; no driving or inspection acknowledgment sent.'%len(commands))
            except Exception as exc:
                clear=Marker();clear.action=Marker.DELETEALL
                self.pub.publish(MarkerArray([clear]))
                self.status.publish('INVALID PLAN: '+str(exc));rospy.logerr(str(exc))

if __name__=='__main__':
    rospy.init_node('inspection_preview');Preview();rospy.spin()
