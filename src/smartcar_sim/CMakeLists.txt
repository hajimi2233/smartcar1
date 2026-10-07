cmake_minimum_required(VERSION 2.8.3)
project(smartcar_sim)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs sensor_msgs tf)
find_package(gazebo REQUIRED)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_library(inspection_ackermann SHARED src/ackermann_plugin.cpp)
target_link_libraries(inspection_ackermann ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


catkin_install_python(PROGRAMS scripts/laser_noise.py scripts/imu_bridge.py scripts/terminal_teleop.py scripts/reset_to_start.py scripts/turn90_calibration.py scripts/turn_calibration_record.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
