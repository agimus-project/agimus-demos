cmake_minimum_required(VERSION 3.8)
project(test_agimus_type)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "action/PlanBarPick.action")

ament_package()
