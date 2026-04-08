cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_10_tiago_pro_bar_manip LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
find_package(test_agimus_type REQUIRED)
ament_auto_find_build_dependencies()

#
# Build Python package.
#
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
                             lib/${PROJECT_NAME})

#
# Installation
#
# Install the tuck_arm.py script as a ROS node
ament_auto_package(
  INSTALL_TO_SHARE
  urdf
  srdf
  meshes
  config
  launch
  rviz)
