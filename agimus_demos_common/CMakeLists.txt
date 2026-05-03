cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demos_common LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

generate_parameter_module(
  disable_franka_collisions_parameters
  agimus_demos_common/disable_franka_collisions_parameters.yaml
)

ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
                             lib/${PROJECT_NAME}
)

#
# Installation
#
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
    DIRECTORY config launch rviz plotjuggler templates
    DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
