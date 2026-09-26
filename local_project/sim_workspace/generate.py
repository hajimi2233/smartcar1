#!/usr/bin/env python
from __future__ import print_function
import os, sys, json, math, shutil, io
import xml.etree.ElementTree as ET

HERE=os.path.dirname(os.path.abspath(__file__))
WS=os.path.expanduser(sys.argv[1] if len(sys.argv)>1 else '~/smartcar_2026_ws')
with open(os.path.join(HERE,'config','simulation.json')) as f: cfg=json.load(f)
v=cfg['vehicle'];g=cfg['field']
def write(path,content):
    dest=os.path.join(WS,path);parent=os.path.dirname(dest)
    if not os.path.isdir(parent):os.makedirs(parent)
    with io.open(dest,'w',encoding='utf-8') as f:f.write(content if isinstance(content,type(u'')) else content.decode('utf-8'))
def package(name,deps,cmake=None):
    xml='<package format="2"><name>%s</name><version>0.1.0</version><description>Inspection simulation integration</description><maintainer email="smartcar@example.com">Project team</maintainer><license>TODO</license><buildtool_depend>catkin</buildtool_depend>'%name
    for d in deps:xml+='<depend>%s</depend>'%d
    xml+='</package>'
    write('src/'+name+'/package.xml',xml)
    write('src/'+name+'/CMakeLists.txt',cmake or 'cmake_minimum_required(VERSION 2.8.3)\nproject(%s)\nfind_package(catkin REQUIRED)\ncatkin_package()\n'%name)
def xmlstr(root):return ET.tostring(root,encoding='utf-8').decode('utf-8')
def e(parent,tag,text=None,**attrs):
    node=ET.SubElement(parent,tag,{k:str(val) for k,val in attrs.items()})
    if text is not None:node.text=str(text)
    return node
def pose(parent,xyz):e(parent,'pose',' '.join(str(x) for x in xyz))

if not(0<v['wheelbase']<v['length'] and 0<v['max_wheel_angle_deg']<80):raise ValueError('Invalid vehicle dimensions or steering limit')
pitch=g['lane_clear_width']+g['wall_thickness_assumed']
xs=[(c-2)*pitch for c in range(5)]
wall_x=[xs[0]-pitch/2+k*pitch for k in range(6)]
end_y=g['channel_length_assumed']/2
portal=end_y+g['portal_clearance_assumed']
side_x=abs(wall_x[0])+1.1
if g['outer_width']/2<side_x+.9 or g['outer_height']/2<portal+.7:raise ValueError('Outer area too small for this configured preview; enlarge outer_width/height')
start=(side_x,portal);finish=(side_x,-portal)
points=[]
for c,x in enumerate(xs):
    points.extend([('col%d_top_entry'%(c+1),x,portal),('col%d_bottom_entry'%(c+1),x,-portal)])
for c,x in enumerate(xs):
    points.extend([('inspect_%d'%(2*c+1),x,g['inspection_y_assumed']),('inspect_%d'%(2*c+2),x,-g['inspection_y_assumed'])])
points.extend([('map_left_top',-side_x,portal),('map_left_bottom',-side_x,-portal),('map_right_top',side_x,portal),('map_right_bottom',side_x,-portal),('start',start[0],start[1]),('end',finish[0],finish[1])])
batch='data/maps/sim_field_v1'
write(batch+'/points.csv','point_id,x,y\n'+''.join('%s,%.6f,%.6f\n'%p for p in points))
write(batch+'/map_info.txt','map_id=sim_field\nmap_version=sim_v1\nframe_id=sim_world\nunits=m\ntarget_reference=front_axle_midpoint\n')
write(batch+'/simulation.json',json.dumps(cfg,indent=2)+'\n')

package('smartcar_description',[])
package('smartcar_mapping',[])
package('smartcar_navigation',[])
package('smartcar_bringup',[])
package('smartcar_sim',['roscpp','geometry_msgs','nav_msgs','sensor_msgs','tf','gazebo_ros','rospy','robot_state_publisher','map_server'],'''cmake_minimum_required(VERSION 2.8.3)
project(smartcar_sim)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs tf)
find_package(gazebo REQUIRED)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(inspection_ackermann SHARED src/ackermann_plugin.cpp)
target_link_libraries(inspection_ackermann ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
catkin_install_python(PROGRAMS scripts/teleop.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
''')
package('smartcar_inspection',['rospy','std_msgs','visualization_msgs','geometry_msgs'],'''cmake_minimum_required(VERSION 2.8.3)
project(smartcar_inspection C CXX)
find_package(catkin REQUIRED)
catkin_package()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11 -Wall -Wextra")
include_directories(core)
add_library(inspection_core core/planner.c core/motion.c core/map_import.c core/session.c)
target_link_libraries(inspection_core m)
add_executable(inspection_planner core/main.c)
target_link_libraries(inspection_planner inspection_core)
add_executable(inspection_route_tests core/tests.c)
target_link_libraries(inspection_route_tests inspection_core)
add_executable(inspection_motion_tests core/motion_tests.c)
target_link_libraries(inspection_motion_tests inspection_core)
catkin_install_python(PROGRAMS scripts/preview.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
''')
for name,dest in [('ackermann_plugin.cpp','src/smartcar_sim/src/ackermann_plugin.cpp'),('teleop.py','src/smartcar_sim/scripts/teleop.py'),('preview.py','src/smartcar_inspection/scripts/preview.py')]:
    with io.open(os.path.join(HERE,'templates',name),encoding='utf-8') as f:write(dest,f.read())
for dest in ['src/smartcar_sim/scripts/teleop.py','src/smartcar_inspection/scripts/preview.py']:os.chmod(os.path.join(WS,dest),0o755)
core=os.path.join(HERE,'inspection_core')
for name in ['planner.h','planner.c','motion.h','motion.c','map_import.c','session.c','main.c','tests.c','motion_tests.c']:
    with io.open(os.path.join(core,name),encoding='utf-8-sig') as f:write('src/smartcar_inspection/core/'+name,f.read())

# World: walls are solid; overhead picture bands and inspection floor strips are visual only.
root=ET.Element('sdf',version='1.6');world=e(root,'world',name='default')
physics=e(world,'physics',name='default_physics',type='ode');e(physics,'max_step_size',.004);e(physics,'real_time_update_rate',250)
solver=e(e(physics,'ode'),'solver');e(solver,'type','quick');e(solver,'iters',50);e(solver,'sor',1.0)
scene=e(world,'scene');e(scene,'shadows','false');e(scene,'ambient','.65 .65 .65 1')
light=e(world,'light',name='sun',type='directional');pose(light,[0,0,10,0,0,0]);e(light,'diffuse','.8 .8 .8 1');e(light,'direction','-.3 -.2 -1');e(light,'cast_shadows','false')
rects=[]
def box(name,x,y,z,sx,sy,sz,color,collision=True):
    m=e(world,'model',name=name);e(m,'static','true');pose(m,[x,y,z,0,0,0]);ln=e(m,'link',name='body')
    vis=e(ln,'visual',name='visual');geo=e(vis,'geometry');e(e(geo,'box'),'size','%g %g %g'%(sx,sy,sz));mat=e(vis,'material');e(mat,'ambient',color);e(mat,'diffuse',color);e(vis,'cast_shadows','false')
    if collision:
        geo=e(e(ln,'collision',name='collision'),'geometry');e(e(geo,'box'),'size','%g %g %g'%(sx,sy,sz))
        if z-sz/2<.38<z+sz/2:rects.append((x-sx/2,x+sx/2,y-sy/2,y+sy/2))
box('floor',0,0,-.05,g['outer_width']+2,g['outer_height']+2,.1,'.7 .7 .7 1')
wt=g['wall_thickness_assumed'];wh=g['wall_height_assumed']
for i,x in enumerate(wall_x):box('channel_wall_%d'%i,x,0,wh/2,wt,2*end_y,wh,'.15 .3 .7 1')
for sign in [-1,1]:
    box('outer_x_%d'%sign,sign*g['outer_width']/2,0,wh/2,wt,g['outer_height'],wh,'.2 .3 .6 1')
    box('outer_y_%d'%sign,0,sign*g['outer_height']/2,wh/2,g['outer_width'],wt,wh,'.2 .3 .6 1')
divider_left=side_x+.8
box('entrance_exit_divider',(divider_left+g['outer_width']/2)/2,0,wh/2,g['outer_width']/2-divider_left,wt,wh,'.15 .3 .7 1')
for c,x in enumerate(xs):
    for sign in [-1,1]:
        box('inspection_%d'%(2*c+(1 if sign==1 else 2)),x,sign*g['inspection_y_assumed'],.003,g['lane_clear_width'],g['inspection_depth_assumed'],.006,'.2 .7 .7 1',False)
        box('overhead_visual_%d_%d'%(c,sign),x,sign*g['inspection_y_assumed'],1.3,g['lane_clear_width'],.035,.025,'0 .7 1 1',False)
gui=e(world,'gui');camera=e(gui,'camera',name='user_camera');pose(camera,[8,-10,11,0,.68,2.2])
write('src/smartcar_sim/worlds/inspection.world',xmlstr(root))

# A geometry-derived map for RViz/testing. It is not a Cartographer mapping result.
res=g['map_resolution'];xmin=-g['outer_width']/2-.2;ymin=-g['outer_height']/2-.2
nx=int(math.ceil((g['outer_width']+.4)/res));ny=int(math.ceil((g['outer_height']+.4)/res))
rows=[]
for row in range(ny):
    y=ymin+(ny-row-.5)*res;values=[]
    for col in range(nx):
        x=xmin+(col+.5)*res
        occupied=any(x+res/2>=a and x-res/2<=b and y+res/2>=c and y-res/2<=d for a,b,c,d in rects)
        values.append('0' if occupied else '254')
    rows.append(' '.join(values))
write(batch+'/map.pgm','P2\n%d %d\n255\n'%(nx,ny)+'\n'.join(rows)+'\n')
write(batch+'/map.yaml','image: map.pgm\nresolution: %g\norigin: [%g, %g, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'%(res,xmin,ymin))

# URDF reference frame: chassis centre projected onto ground, with an explicit front-axle frame.
robot=ET.Element('robot',name='inspection_car')
def link(name,mass,shape,size,z=0,color='.15 .35 .75 1'):
    ln=e(robot,'link',name=name)
    for kind in ['visual','collision']:
        node=e(ln,kind);e(node,'origin',xyz='0 0 %g'%z,rpy='0 0 0');geo=e(node,'geometry')
        if shape=='box':e(geo,'box',size=' '.join(str(a) for a in size))
        else:
            node.find('origin').set('rpy','1.57079632679 0 0');e(geo,'cylinder',radius=size[0],length=size[1])
        if kind=='visual':e(e(node,'material',name=name+'_mat'),'color',rgba=color)
    inert=e(ln,'inertial');e(inert,'origin',xyz='0 0 %g'%z);e(inert,'mass',value=mass)
    if shape=='box':a,b,c=size;ix=mass*(b*b+c*c)/12;iy=mass*(a*a+c*c)/12;iz=mass*(a*a+b*b)/12
    else:r,w=size;iy=.5*mass*r*r;ix=iz=mass*(3*r*r+w*w)/12
    e(inert,'inertia',ixx=ix,iyy=iy,izz=iz,ixy=0,ixz=0,iyz=0)
    gz=e(robot,'gazebo',reference=name);e(gz,'material','Gazebo/Black' if shape=='cylinder' else 'Gazebo/Blue');e(gz,'mu1',1);e(gz,'mu2',1);e(gz,'selfCollide','false')
    return ln
def joint(name,typ,parent,child,xyz,axis=None):
    j=e(robot,'joint',name=name,type=typ);e(j,'parent',link=parent);e(j,'child',link=child);e(j,'origin',xyz=' '.join(str(a) for a in xyz))
    if axis:e(j,'axis',xyz=axis)
    return j
e(robot,'link',name='base_footprint')
link('base_link',v['body_mass_assumed'],'box',[v['length'],v['width'],v['body_height_assumed']])
joint('base_fixed','fixed','base_footprint','base_link',[0,0,v['body_center_z_assumed']])
e(robot,'link',name='front_axle_midpoint')
joint('front_axle_fixed','fixed','base_link','front_axle_midpoint',[v['wheelbase']/2,0,-v['body_center_z_assumed']])
for front in [True,False]:
    for left in [True,False]:
        name=('front' if front else 'rear')+'_'+('left' if left else 'right')
        x=(1 if front else -1)*v['wheelbase']/2;y=(1 if left else -1)*v['front_track' if front else 'rear_track']/2
        z=v['wheel_diameter']/2-v['body_center_z_assumed'];parent='base_link'
        if front:
            link(name+'_knuckle',.05,'box',[.025,.025,.025]);j=joint(name+'_steer_joint','revolute',parent,name+'_knuckle',[x,y,z],'0 0 1')
            limit=math.radians(v['max_wheel_angle_deg']);e(j,'limit',lower=-limit,upper=limit,effort=20,velocity=v['steer_rate_assumed']);parent=name+'_knuckle'
        link(name+'_wheel',v['wheel_mass_assumed'],'cylinder',[v['wheel_diameter']/2,v['wheel_width']],color='.1 .1 .1 1')
        joint(name+'_wheel_joint','continuous',parent,name+'_wheel',[0,0,0] if front else [x,y,z],'0 1 0')
link('laser',.1,'box',[.065,.065,.04],color='.8 .1 .1 1')
joint('laser_fixed','fixed','base_link','laser',[v['laser_x_from_body_center_assumed'],v['laser_y_assumed'],v['laser_z_assumed']-v['body_center_z_assumed']])
gz=e(robot,'gazebo',reference='laser');sensor=e(gz,'sensor',type='ray',name='inspection_lidar');e(sensor,'always_on','true');e(sensor,'visualize','false');e(sensor,'update_rate',v['laser_hz'])
ray=e(sensor,'ray');horizontal=e(e(ray,'scan'),'horizontal');e(horizontal,'samples',v['laser_samples_test']);e(horizontal,'resolution',1);e(horizontal,'min_angle',-math.pi);e(horizontal,'max_angle',math.pi)
ran=e(ray,'range');e(ran,'min',v['laser_min_range']);e(ran,'max',v['laser_max_range']);e(ran,'resolution',.01)
plugin=e(sensor,'plugin',name='laser_ros',filename='libgazebo_ros_laser.so');e(plugin,'robotNamespace','/');e(plugin,'topicName','/sim/scan');e(plugin,'frameName','laser')
plugin=e(e(robot,'gazebo'),'plugin',name='four_wheel_ackermann',filename='libinspection_ackermann.so')
for key,value in [('wheelbase',v['wheelbase']),('front_track',v['front_track']),('rear_track',v['rear_track']),('wheel_radius',v['wheel_diameter']/2),('max_wheel_angle',math.radians(v['max_wheel_angle_deg'])),('wheel_torque',v['drive_torque_assumed']),('steer_torque',20.0),('steer_kp',8.0),('max_speed',v['max_speed_test']),('acceleration',v['acceleration_test']),('braking',0.5),('steer_rate',v['steer_rate_assumed']),('publish_tf',0)]:e(plugin,key,value)
imu_pl=e(e(robot,'gazebo'),'plugin',name='imu_plugin',filename='libgazebo_ros_imu.so')
e(imu_pl,'alwaysOn','true');e(imu_pl,'updateRate',50);e(imu_pl,'bodyName','base_link')
e(imu_pl,'topicName','/sim/imu');e(imu_pl,'frameName','base_link');e(imu_pl,'gaussianNoise',0.002)
e(imu_pl,'xyzOffset','0 0 0');e(imu_pl,'rpyOffset','0 0 0')
write('src/smartcar_description/urdf/inspection_car.urdf',xmlstr(robot))

write('src/smartcar_bringup/launch/simulation.launch','''<launch>
  <arg name="gui" default="true"/><arg name="rviz" default="true"/>
  <arg name="layout" default="%s"/>
  <arg name="data_dir" default="%s"/>
  <env name="GAZEBO_MODEL_DATABASE_URI" value=""/>
  <param name="use_sim_time" value="true"/>
  <rosparam file="$(find smartcar_sim)/config/vehicle.yaml" ns="sim"/>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find smartcar_sim)/worlds/inspection.world"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>
  <param name="robot_description" textfile="$(find smartcar_description)/urdf/inspection_car.urdf"/>
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_car" args="-urdf -param robot_description -model inspection_car -x %g -y %g -z 0.015 -Y 3.141592653589793" output="screen"/>
  <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher">
    <remap from="joint_states" to="/sim/joint_states"/>
  </node>
  <node pkg="map_server" type="map_server" name="sim_reference_map" args="$(arg data_dir)/map.yaml"><param name="frame_id" value="sim_world"/></node>
  <node pkg="smartcar_inspection" type="preview.py" name="inspection_preview" output="screen">
    <param name="map_info" value="$(arg data_dir)/map_info.txt"/><param name="points" value="$(arg data_dir)/points.csv"/>
    <param name="output" value="%s"/><param name="layout" value="$(arg layout)"/>
  </node>
  <node if="$(arg rviz)" pkg="rviz" type="rviz" name="inspection_rviz" args="-d $(find smartcar_sim)/config/inspection.rviz">
    <env name="OGRE_RTT_MODE" value="Copy"/>
    <env name="QT_X11_NO_MITSHM" value="1"/>
    <env name="QT_AUTO_SCREEN_SCALE_FACTOR" value="0"/>
  </node>
</launch>
'''%(cfg['layout_example'],os.path.join(WS,batch),start[0]+v['wheelbase']/2,start[1],os.path.join(WS,'data/logs/inspection_preview')))
write('src/smartcar_sim/config/vehicle.yaml','wheelbase: %g\n'%v['wheelbase'])
half_l=v['length']/2;half_w=v['width']/2
angles=[math.radians(v['max_wheel_angle_deg'])*i/1000 for i in range(1001)]
sweep_x=v['wheelbase']/2+max(v['wheel_diameter']/2*math.cos(a)+v['wheel_width']/2*math.sin(a) for a in angles)
sweep_y=v['front_track']/2+max(v['wheel_diameter']/2*math.sin(a)+v['wheel_width']/2*math.cos(a) for a in angles)
def footprint(x,y,offset=0):return [[-x-offset,-y],[x-offset,-y],[x-offset,y],[-x-offset,y]]
geometry={'straight_body_footprint':footprint(half_l,half_w),
          'front_axle_body_footprint':footprint(half_l,half_w,v['wheelbase']/2),
          'turning_wheel_envelope':footprint(max(half_l,sweep_x),max(half_w,sweep_y)),
          'note':'Geometry only, no safety inflation or navigation controller configured.'}
write('src/smartcar_navigation/config/measured_geometry.yaml',json.dumps(geometry,indent=2)+'\n')
write('src/smartcar_sim/config/inspection.rviz','''Panels:
  - Class: rviz/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Global Options:
    Fixed Frame: sim_world
    Background Color: 40; 40; 45
    Frame Rate: 10
  Displays:
    - Class: rviz/Grid
      Name: Grid
      Enabled: true
      Cell Size: 1
      Plane Cell Count: 20
    - Class: rviz/Map
      Name: Geometry reference map (not SLAM)
      Enabled: true
      Topic: /map
      Alpha: 0.25
    - Class: rviz/RobotModel
      Name: Measured car dimensions
      Enabled: false
      Robot Description: robot_description
    - Class: rviz/LaserScan
      Name: Sim lidar
      Enabled: true
      Topic: /sim/scan
      Size (m): 0.06
      Style: Squares
      Color Transformer: FlatColor
      Color: 255; 30; 30
    - Class: rviz/MarkerArray
      Name: Inspection points (plan only)
      Enabled: true
      Marker Topic: /inspection/preview
  Views:
    Current:
      Class: rviz/TopDownOrtho
      Name: Top down
      Scale: 60
      X: 0
      Y: 0
Window Geometry:
  Width: 1100
  Height: 720
''')
write('scripts/source_env.sh','''# Source this file from Bash; no .bashrc changes required.
source /opt/ros/kinetic/setup.bash
_inspection_ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -f "$_inspection_ws/devel/setup.bash" ]; then source "$_inspection_ws/devel/setup.bash"; fi
export GAZEBO_PLUGIN_PATH="$_inspection_ws/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
unset _inspection_ws
''')
write('scripts/build.sh','''#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
catkin_make -j2 -l2
''')
with io.open(os.path.join(HERE,'templates','run_sim.sh'),encoding='utf-8') as f:
    write('scripts/run_sim.sh',f.read())
write('scripts/keyboard.sh','''#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
source scripts/source_env.sh
export DISPLAY="${DISPLAY:-:0}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
exec rosrun smartcar_sim teleop.py
''')
write('src/smartcar_navigation/README.md','Navigation execution is deferred: no move_base launch or action server is started. TEB forward/reverse constraints and forbidden-area handling require integration and tests. Do not connect the real vehicle /cmd_vel directly to /sim/cmd_vel.\n')
write('src/smartcar_mapping/README.md','Cartographer is not installed or configured by this generator. data/maps/sim_field_v1/map.yaml is generated from world wall geometry, not from lidar mapping.\n')
print('Generated workspace at '+WS)
