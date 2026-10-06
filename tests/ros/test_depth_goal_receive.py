#!/usr/bin/env python
"""Use actual ROS message classes and Navigator callbacks without publishers."""
from __future__ import print_function
import json
import math
import sys
import threading
sys.path.insert(0, sys.argv[1])
from single_goal_nav import Navigator
from ackermann_core import Grid, rear_target
from nav_config import configure
from std_msgs.msg import String

configure()
regions=[dict(id='inspect_%d'%(i+1),type='B' if i==8 else 'A',x=2.-i//2,y=1. if i%2==0 else -1.)
         for i in range(10)]
for virtual in (False, True):
    n=Navigator.__new__(Navigator)
    n.lock=threading.RLock(); n.inspection_regions=[]; n.inspection_sides={}
    n.grid=Grid(200,200,.1,(-10.,-10.,0.),[0]*40000)
    n.ground_truth_test=virtual
    n.sensors=lambda:None; n.authority=lambda:None; n.calibrate_truth=lambda:None
    statuses=[]; n.status=statuses.append; n.halt=statuses.append
    b=regions[8]
    n.pose=lambda:rear_target((b['x'],b['y']-1.,math.pi/2))
    def send(x,y,yaw,target):
        n.on_inspection_goal(String(data=json.dumps(dict(frame_id='map',x=x,y=y,yaw=yaw,
                                                        target_id=target,regions=regions))))
        assert not any(s.startswith('REJECTED') for s in statuses), statuses
    send(b['x'],b['y'],math.pi/2,'inspect_9')
    assert n.state=='WAIT_STOP' and n.grid.depth_rules[0]['direction']==1
    # Arrival at B9, then end: retain B9's depth cap without generating exit goals.
    n.pose=lambda:rear_target((b['x'],b['y'],math.pi/2))
    send(5.,-2.,0.,None)
    assert len(n.grid.depth_rules)==1 and n.grid.depth_rules[0]['id']=='inspect_9'
    assert not n.grid.free(rear_target((b['x'],b['y']+.5,math.pi/2)))
    assert n.grid.arc(n.pose(),-.6,0.) is not None
    # A new A target adds its own cap; it must not erase current B occupancy.
    a=regions[0]
    send(a['x'],a['y'],-math.pi/2,'inspect_1')
    assert len(n.grid.depth_rules)==2
    assert not n.grid.free(rear_target((a['x'],a['y']-.5,-math.pi/2)))
    print('PASS: atomic goal -> Navigator -> collision rules, mode='+('nav-test' if virtual else 'nav'))
