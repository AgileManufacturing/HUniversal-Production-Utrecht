cmake_minimum_required(VERSION 2.8.3)
project(gripper_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rexos_module rexos_modbus rexos_utilities rexos_datatypes rexos_gripper rexos_logger)
find_package(Jsoncpp)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS roscpp rexos_module rexos_modbus rexos_utilities rexos_datatypes rexos_gripper rexos_logger
DEPENDS Jsoncpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gripper_node src/GripperNode.cpp)
target_link_libraries(gripper_node ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})