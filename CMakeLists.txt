cmake_minimum_required(VERSION 3.4)
project(agimus_demos_description)

find_package(catkin REQUIRED)

install(DIRECTORY pick_and_place
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
