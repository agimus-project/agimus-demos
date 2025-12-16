cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_04_dual_arm_tiago_pro LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
# Installation
#
# Install the tuck_arm.py script as a ROS node
ament_auto_package(INSTALL_TO_SHARE urdf srdf meshes)
