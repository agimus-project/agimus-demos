cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_07_deburring LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
# Build Python package.
#
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/deburring_orchestrator_node
        DESTINATION lib/${PROJECT_NAME})

generate_parameter_module(orchestrator_parameters
                          agimus_demo_07_deburring/orchestrator_parameters.yaml)

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
ament_auto_package(INSTALL_TO_SHARE config launch urdf srdf rviz)
