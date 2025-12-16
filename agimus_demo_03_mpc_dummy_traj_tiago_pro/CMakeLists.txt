cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_03_mpc_dummy_traj_tiago_pro LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
# Build Python package.
#
ament_python_install_package(${PROJECT_NAME})

#
# Installation
#

# Install ROS2 nodes.
install(PROGRAMS scripts/tuck_arm.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/orchestrator_node.py DESTINATION lib/${PROJECT_NAME})
# Install ament and share folders.
ament_auto_package(INSTALL_TO_SHARE config launch)
