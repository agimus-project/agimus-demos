cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_10_tiago_pro_bar_manip LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(test_agimus_type REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)
ament_auto_find_build_dependencies()

#
# Build Python package
#
ament_python_install_package(${PROJECT_NAME} SCRIPTS_DESTINATION
                             lib/${PROJECT_NAME})

#
# RViz plugin
#
add_library(hpp_plugin SHARED src/hpp_plugin.cpp)

target_include_directories(
  hpp_plugin PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(hpp_plugin rclcpp rclcpp_action rviz_common pluginlib
                          test_agimus_type)

target_link_libraries(hpp_plugin Qt5::Widgets Qt5::Core)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(
  TARGETS hpp_plugin
  EXPORT export_hpp_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)

install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})

#
# Installation
#
ament_auto_package(
  INSTALL_TO_SHARE
  urdf
  srdf
  meshes
  config
  launch
  rviz)
