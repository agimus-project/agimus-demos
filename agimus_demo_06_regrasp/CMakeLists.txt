cmake_minimum_required(VERSION 3.22.1)

#
# Project definition
#
project(agimus_demo_06_regrasp LANGUAGES CXX)

#
# Handle dependencies by reading the package.xml
#
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

#
# Build Python package.
#
ament_python_install_package(${PROJECT_NAME})
install(
    PROGRAMS scripts/regrasp_node scripts/image_saver
    DESTINATION lib/${PROJECT_NAME}
)

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
ament_auto_package(INSTALL_TO_SHARE config launch urdf srdf)
