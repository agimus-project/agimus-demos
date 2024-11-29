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

#
# Installation
#
# install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()