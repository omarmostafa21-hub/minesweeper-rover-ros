cmake_minimum_required(VERSION 3.0.2)
project(minesweeper_core)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf)
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs sensor_msgs tf)
include_directories(${catkin_INCLUDE_DIRS})
catkin_install_python(PROGRAMS
  scripts/odometry_node.py
  scripts/grid_map_node.py
  scripts/teleop_ps4_node.py
  scripts/live_grid_map_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
