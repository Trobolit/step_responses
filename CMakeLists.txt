cmake_minimum_required(VERSION 3.0)
project(step_responses)

find_package(catkin_simple REQUIRED)

catkin_simple()

cs_add_executable(${PROJECT_NAME} src/torque_step_sysid.cpp)

cs_install()

cs_export()

