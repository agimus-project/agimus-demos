cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_03_mpc_dummy_traj LANGUAGES CXX)


#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()