cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_08_polishing LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set_property(
  DIRECTORY
  APPEND
  PROPERTY
    CMAKE_CONFIGURE_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/agimus_demo_08_polishing/planner/polishing_path_planner_parameters.yaml
)

#
# Build Python package.
#

generate_parameter_module(
  polishing_path_planner_parameters
  agimus_demo_08_polishing/planner/polishing_path_planner_parameters.yaml)

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
ament_auto_package(INSTALL_TO_SHARE config launch rviz urdf environment)
