#!/usr/bin/env python
"""Bind an explicit copied line fixture to the isolated test map only."""
import json,sys
import rospy
from nav_msgs.msg import OccupancyGrid
sys.path.insert(0,'/home/hajimi/smartcar_ws/src/smartcar_navigation/scripts')
from low_cost_lines import LineStore,map_key,validate
rospy.init_node('bind_test_lines')
m=rospy.wait_for_message('/map',OccupancyGrid,timeout=40)
with open('/tmp/lines-source.json') as f: data=json.load(f)
sets=list(data['maps'].values())
if len(sets)!=1:raise ValueError('expected one explicit line fixture')
lines=validate(sets[0])
if len(lines)!=2:raise ValueError('expected the two user-drawn line segments')
# Check the copied segments lie on known free map cells. Interior wall geometry
# is unchanged between test maps; their surrounding outer space differs.
import math
c,s=math.cos(0.),math.sin(0.)
if abs(m.info.origin.orientation.z)>1e-6:raise ValueError('expected axis-aligned map')
for a,b in lines:
    n=int(math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/m.info.resolution))+1
    for i in range(n+1):
        x=a[0]+(b[0]-a[0])*i/n;y=a[1]+(b[1]-a[1])*i/n
        ix=int(math.floor((x-m.info.origin.position.x)/m.info.resolution))
        iy=int(math.floor((y-m.info.origin.position.y)/m.info.resolution))
        if not (0<=ix<m.info.width and 0<=iy<m.info.height) or m.data[iy*m.info.width+ix]!=0:
            raise ValueError('copied line is outside free map cells')
LineStore('/tmp/lines.json').save(map_key(m),lines)
print('BOUND_TEST_COPY: two unchanged line segments, map='+map_key(m))
