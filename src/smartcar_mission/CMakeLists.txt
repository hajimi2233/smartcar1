cmake_minimum_required(VERSION 2.8.3)
project(smartcar_mission)
find_package(catkin REQUIRED)
catkin_package()
add_executable(inspection_plan14 core/planner.c core/plan14.c)
set_property(TARGET inspection_plan14 PROPERTY COMPILE_FLAGS "-std=c99 -Wall -Wextra")
target_link_libraries(inspection_plan14 m)
install(TARGETS inspection_plan14 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS
  scripts/multi_goal_nav.py
  scripts/inspection_plan.py
  scripts/inspection_plan_test.py
  scripts/plan_io.py
  scripts/inspection_points.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
