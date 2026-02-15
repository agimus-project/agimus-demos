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
# Download weights of diffusion model
#
# TODO change URL to a more permanent one and not @Kotochleb personal partage
file(
  DOWNLOAD
  "https://partage.laas.fr/public.php/dav/files/N632KTiLi6gyaDc/?accept=zip"
  "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/weights/weights.ckpt"
  SHOW_PROGRESS)

#
# Build Python package.
#

generate_parameter_module(
  deburring_path_planner_parameters
  agimus_demo_07_deburring/planner/deburring_path_planner_parameters.yaml)

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
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  # TODO add configs from Kurai rviz
  plotjuggler
  precomputed_trajectories
  urdf
  environment)
