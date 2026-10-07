#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Terminal-scoped, gear-based commands. No global X11 keyboard hooks."""
from __future__ import print_function
import sys, select, termios, tty, time, math
import rospy
from geometry_msgs.msg import Twist

def main():
    if not sys.stdin.isatty():
        raise RuntimeError('Run with an interactive terminal (docker compose exec, no -T).')
    rospy.init_node('inspection_sim_keyboard')
    pub = rospy.Publisher('/sim/cmd_vel', Twist, queue_size=1)
    old = termios.tcgetattr(sys.stdin)
    gear, steering = 0, 0.0
    # Gears -4..-1 are reverse, 0 is stopped, and 1..4 are forward.
    gear_speeds = {-4: -0.35, -3: -0.26, -2: -0.18, -1: -0.10,
                   0: 0.0, 1: 0.10, 2: 0.18, 3: 0.26, 4: 0.35}
    wheelbase = 0.62
    min_turn_radius = 1.30
    steer_step = math.radians(20.0)
    # Ackermann curvature: R = wheelbase / tan(delta).
    max_steering = math.atan(wheelbase / min_turn_radius)
    print('W: forward one gear; S: reverse one gear (four gears each direction).')
    print('A/D: steer +/-20 degrees (limit +/-{:.1f}, R >= {:.2f} m). X center, SPACE stop, Q exit.'.format(
        math.degrees(max_steering), min_turn_radius))
    print('Focus this terminal. Speed and steering remain until changed or stopped.')
    try:
        tty.setcbreak(sys.stdin.fileno())
        while not rospy.is_shutdown():
            if select.select([sys.stdin], [], [], 0.03)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'q': break
                if key == 'w': gear = min(4, gear + 1)
                elif key == 's': gear = max(-4, gear - 1)
                elif key == 'a': steering = min(max_steering, steering + steer_step)
                elif key == 'd': steering = max(-max_steering, steering - steer_step)
                elif key == 'x': steering = 0.0
                elif key == ' ': gear = 0
            msg = Twist()
            msg.linear.x = gear_speeds[gear]
            msg.angular.z = msg.linear.x * math.tan(steering) / wheelbase
            pub.publish(msg)
    finally:
        for _ in range(3):
            pub.publish(Twist()); time.sleep(0.03)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

if __name__ == '__main__':
    main()
