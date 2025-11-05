cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_08_collision_avoidance LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
# Build Python package.
#

generate_parameter_module(
  goal_publisher_parameters
  agimus_demo_08_collision_avoidance/goal_publisher_parameters.yaml)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
                             lib/${PROJECT_NAME})

#
# Unit tests
#
include(CTest)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_auto_find_test_dependencies()
  # Integration test of the roscontrol controller with simulation on Talos.
  # add_rostest(tests/test_integration.py)
endif()

#
# Installation
#
ament_auto_package(INSTALL_TO_SHARE config launch rviz urdf)
