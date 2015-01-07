cmake_minimum_required(VERSION 2.8.3)
project(rexos_gripper)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_modbus rexos_utilities rexos_logger)
find_package(Boost)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_gripper rexos_logger
CATKIN_DEPENDS rexos_modbus rexos_utilities rexos_logger
DEPENDS Boost)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_gripper ${sources})

target_link_libraries(rexos_gripper ${catkin_LIBRARIES} ${Boost_LIBRARIES})



