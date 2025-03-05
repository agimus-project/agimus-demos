cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_02_simple_pd_plus_tiago_pro LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
# Installation
#
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()
