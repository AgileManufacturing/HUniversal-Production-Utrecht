cmake_minimum_required(VERSION 2.8.3)
project(delta_robot_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rexos_statemachine rexos_modbus rexos_utilities rexos_datatypes rexos_delta_robot rexos_coordinates equiplet_node matrices rexos_logger)
find_package(Jsoncpp)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS roscpp rexos_statemachine rexos_modbus rexos_utilities rexos_datatypes rexos_delta_robot rexos_coordinates equiplet_node matrices rexos_logger
DEPENDS Jsoncpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(delta_robot_node src/DeltaRobotNode.cpp)
target_link_libraries(delta_robot_node ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})