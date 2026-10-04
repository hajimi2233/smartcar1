#!/usr/bin/env python
"""SIMULATION ONLY: truth -> quantized scalar pulses. Never publishes pose or TF."""
import uuid
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from wheel_distance import PulseSimulator


class Encoder(object):
    def __init__(self):
        self.model=PulseSimulator(float(rospy.get_param('~ticks_per_meter',10000.)),
             float(rospy.get_param('~scale_error',.05)),float(rospy.get_param('~rear_offset',.31)))
        self.run_id=uuid.uuid4().hex
        self.publisher=rospy.Publisher('/wheel/pulses',JointState,queue_size=100)
        rospy.Subscriber('/sim/ground_truth/odom',Odometry,self.update,queue_size=100)

    def update(self,msg):
        p=msg.pose.pose;q=p.orientation
        yaw=tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2]
        ticks=self.model.update(msg.header.stamp.to_sec(),(p.position.x,p.position.y,yaw))
        result=JointState();result.header.stamp=msg.header.stamp
        result.header.frame_id='encoder_'+self.run_id+'_'+str(self.model.epoch)
        result.name=['rear_center_encoder_ticks'];result.position=[ticks]
        self.publisher.publish(result)


if __name__=='__main__':
    rospy.init_node('sim_wheel_pulses');Encoder();rospy.spin()
