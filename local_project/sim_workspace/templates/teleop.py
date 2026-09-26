#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import os, sys, time, math, ctypes, ctypes.util
import rospy
from geometry_msgs.msg import Twist

class XKeys(object):
    def __init__(self):
        lib = ctypes.util.find_library('X11')
        if not lib:
            raise RuntimeError('libX11 not found')
        self.x11 = ctypes.cdll.LoadLibrary(lib)
        self.x11.XOpenDisplay.restype = ctypes.c_void_p
        self.x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
        self.x11.XKeysymToKeycode.restype = ctypes.c_uint
        self.x11.XKeysymToKeycode.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
        self.x11.XQueryKeymap.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
        self.x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
        name = os.environ.get('DISPLAY', ':0')
        if not isinstance(name, bytes):
            name = name.encode('utf-8')
        self.dpy = self.x11.XOpenDisplay(name)
        if not self.dpy:
            raise RuntimeError('XOpenDisplay failed for %s' % name)
        def kc(sym):
            code = self.x11.XKeysymToKeycode(self.dpy, sym)
            if not code:
                raise RuntimeError('no keycode for 0x%x' % sym)
            return code
        self.w = kc(0x0077)
        self.a = kc(0x0061)
        self.s = kc(0x0073)
        self.d = kc(0x0064)
        self.q = kc(0x0071)
        self.x = kc(0x0078)
        self.space = kc(0x0020)

    def down(self, keycode):
        buf = ctypes.create_string_buffer(32)
        self.x11.XQueryKeymap(self.dpy, buf)
        return bool(ord(buf.raw[keycode // 8]) & (1 << (keycode % 8)))

    def close(self):
        if self.dpy:
            self.x11.XCloseDisplay(self.dpy)
            self.dpy = None

def main():
    rospy.init_node('inspection_sim_keyboard')
    pub = rospy.Publisher('/sim/cmd_vel', Twist, queue_size=1)
    wheelbase = rospy.get_param('/sim/wheelbase', 0.62)
    keys = XKeys()
    delta = 0.0
    max_delta = 0.70
    steer_rate = 2.0
    dt = 0.03
    print('Hold W to drive, release W to stop. Hold S to reverse.')
    print('Hold A/D to steer at the same time. X straighten, Space stop, Q quit.')
    print('Click the Gazebo window first. Do not type WASD in RViz.')
    try:
        while not rospy.is_shutdown():
            if keys.down(keys.q):
                break
            if keys.down(keys.w) and not keys.down(keys.s):
                velocity = 0.25
            elif keys.down(keys.s) and not keys.down(keys.w):
                velocity = -0.25
            else:
                velocity = 0.0
            if keys.down(keys.a) and not keys.down(keys.d):
                delta = min(max_delta, delta + steer_rate * dt)
            elif keys.down(keys.d) and not keys.down(keys.a):
                delta = max(-max_delta, delta - steer_rate * dt)
            if keys.down(keys.x):
                delta = 0.0
            if keys.down(keys.space):
                velocity = 0.0
                delta = 0.0
            m = Twist()
            m.linear.x = velocity
            m.angular.z = velocity * math.tan(delta) / wheelbase
            pub.publish(m)
            time.sleep(dt)
    finally:
        pub.publish(Twist())
        keys.close()

if __name__ == '__main__':
    main()
