cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demos_controllers LANGUAGES CXX)

#
# Options
#
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

generate_parameter_library(ft_calibration_filter_parameters
                           src/ft_calibration_filter_parameters.yaml)
set(${PROJECT_NAME}_headers
    include/${PROJECT_NAME}/visibility.hpp
    include/${PROJECT_NAME}/butterworth_filter.hpp
    include/${PROJECT_NAME}/ft_calibration_filter.hpp)

set(${PROJECT_NAME}_sources src/ft_calibration_filter.cpp)

ament_auto_add_library(${PROJECT_NAME} ${${PROJECT_NAME}_sources}
                       ${${PROJECT_NAME}_headers})

set_target_properties(${PROJECT_NAME} PROPERTIES CXX_VISIBILITY_PRESET hidden)

target_compile_definitions(
  ${PROJECT_NAME}
  PUBLIC -DAGIMUS_DEMOS_CONTROLLERS_IS_SHARED
  PRIVATE -DAGIMUS_DEMOS_CONTROLLERS_DO_EXPORT)

target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)
ament_target_dependencies(${PROJECT_NAME} Eigen3)
target_link_libraries(${PROJECT_NAME} ft_calibration_filter_parameters)

#
# Export plugins
#
pluginlib_export_plugin_description_file(controller_interface
                                         controller_plugins.xml)

#
# Installation
#

ament_export_libraries(${PROJECT_NAME})
ament_auto_package()
