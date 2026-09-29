#!/usr/bin/env python
from __future__ import print_function
import sys, select, termios, tty, time, math
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('inspection_sim_keyboard')
    if not sys.stdin.isatty():
        raise RuntimeError('Run in an interactive terminal, or use ssh -t.')
    pub=rospy.Publisher('/sim/cmd_vel',Twist,queue_size=1)
    wheelbase=rospy.get_param('/sim/wheelbase',0.62)
    previous=termios.tcgetattr(sys.stdin)
    velocity=0.0; delta=0.0; last=time.time()
    print('SIMULATION ONLY: W/S forward/reverse; A/D steering; X straighten; SPACE stop; Q exit.')
    print('Hold W or S to continue. Releasing keys stops after 0.4 seconds.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            if select.select([sys.stdin],[],[],0.05)[0]:
                key=sys.stdin.read(1).lower()
                if key in ('q','\x03',''): break
                if key=='w': velocity=0.20
                elif key=='s': velocity=-0.20
                elif key=='a': delta=min(0.45,delta+0.05)
                elif key=='d': delta=max(-0.45,delta-0.05)
                elif key=='x': delta=0
                elif key==' ': velocity=0;delta=0
                last=time.time()
            if time.time()-last>.4: velocity=0
            m=Twist();m.linear.x=velocity;m.angular.z=velocity*math.tan(delta)/wheelbase
            pub.publish(m)
    finally:
        pub.publish(Twist());termios.tcsetattr(sys.stdin,termios.TCSADRAIN,previous)

if __name__=='__main__': main()
