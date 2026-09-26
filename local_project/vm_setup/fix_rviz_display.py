from __future__ import print_function
import os
import xml.etree.ElementTree as ET
ws=os.path.expanduser('~/smartcar_2026_ws')
cfg=os.path.join(ws,'src/smartcar_sim/config/inspection.rviz')
with open(cfg) as f:s=f.read()
s=s.replace('Alpha: 0.7','Alpha: 0.25').replace('Size (m): 0.025','Size (m): 0.06').replace('Style: Points','Style: Squares').replace('Color: 255; 80; 50','Color: 255; 30; 30')
with open(cfg,'w') as f:f.write(s)
launch=os.path.join(ws,'src/smartcar_bringup/launch/simulation.launch')
tree=ET.parse(launch)
for node in tree.getroot().findall('node'):
    if node.get('name')=='inspection_rviz':
        if not any(x.get('name')=='LIBGL_ALWAYS_SOFTWARE' for x in node.findall('env')):
            ET.SubElement(node,'env',{'name':'LIBGL_ALWAYS_SOFTWARE','value':'1'})
tree.write(launch)
print('RViz display updated; simulation and vehicle state are unchanged.')
