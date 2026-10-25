#!/usr/bin/env python
from __future__ import division
import math
import rospy
import tf
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker,MarkerArray

class Visual(object):
    def __init__(self):
        t=rospy.get_param('/single_goal_nav/effective_tuning',{})
        self.front=float(t.get('body_front',.665625));self.rear=float(t.get('body_rear',.084375))
        self.width=2*float(t.get('body_half_width',.25))
        self.L=float(t.get('wheelbase',.62));self.T=float(t.get('front_track',.445));self.radius=float(t.get('wheel_radius',.09))
        self.frame=t.get('base_frame','base_footprint');self.angles=[0.,0.]
        self.pub=rospy.Publisher('/vehicle/model_markers',MarkerArray,queue_size=1,latch=True)
        rospy.Subscriber('/vehicle/joint_states',JointState,self.joints,queue_size=1)
        self.timer=rospy.Timer(rospy.Duration(.1),self.draw)

    def joints(self,m):
        try:self.angles=[m.position[m.name.index(n)] for n in ('front_left_steer_joint','front_right_steer_joint')]
        except (ValueError,IndexError):pass

    def marker(self,identifier,kind):
        m=Marker();m.header.frame_id=self.frame;m.header.stamp=rospy.Time(0)
        m.ns='vehicle';m.id=identifier;m.type=kind;m.action=Marker.ADD;m.frame_locked=True
        m.pose.orientation.w=1.;m.color.a=1.;return m

    def draw(self,event):
        out=MarkerArray();body=self.marker(0,Marker.CUBE)
        body.pose.position.x=(self.front-self.rear)/2;body.pose.position.z=.17
        body.scale.x=self.front+self.rear;body.scale.y=self.width;body.scale.z=.16
        body.color.r=.15;body.color.g=.45;body.color.b=.90;out.markers.append(body)
        angles=list(self.angles)
        for i,(x,y,angle) in enumerate([(self.L,self.T/2,angles[0]),(self.L,-self.T/2,angles[1]),(0,self.T/2,0),(0,-self.T/2,0)]):
            m=self.marker(i+1,Marker.CYLINDER);m.pose.position.x=x;m.pose.position.y=y;m.pose.position.z=self.radius
            q=tf.transformations.quaternion_from_euler(math.pi/2,0,angle)
            m.pose.orientation.x,m.pose.orientation.y,m.pose.orientation.z,m.pose.orientation.w=q
            m.scale.x=m.scale.y=2*self.radius;m.scale.z=.07
            m.color.r=m.color.g=m.color.b=.12;out.markers.append(m)
        arrow=self.marker(5,Marker.ARROW);arrow.pose.position.z=.31
        arrow.scale.x=.60;arrow.scale.y=.045;arrow.scale.z=.08
        arrow.color.r=1.;arrow.color.g=.85;arrow.color.b=0.;out.markers.append(arrow)
        self.pub.publish(out)

if __name__=='__main__':
    rospy.init_node('vehicle_visual');Visual();rospy.spin()
