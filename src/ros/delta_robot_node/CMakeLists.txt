cmake_minimum_required(VERSION 2.8.3)
project(delta_robot_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rexos_mast rexos_modbus rexos_std_msgs rexos_std_srvs rexos_utilities rexos_datatypes rexos_delta_robot Libjson)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS rexos_mast rexos_modbus rexos_std_msgs rexos_std_srvs rexos_utilities rexos_datatypes rexos_delta_robot Libjson roscpp std_msgs
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} )
add_executable(delta_robot_node src/DeltaRobotNode.cpp)


target_link_libraries(delta_robot_node ${catkin_LIBRARIES})
add_dependencies(delta_robot_node crate_locator_node_gencpp)


