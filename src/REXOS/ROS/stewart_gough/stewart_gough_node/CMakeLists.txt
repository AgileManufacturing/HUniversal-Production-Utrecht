cmake_minimum_required(VERSION 2.8.3)
project(stewart_gough_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rexos_statemachine rexos_modbus rexos_utilities rexos_datatypes rexos_stewart_gough actionlib rexos_coordinates rexos_logger matrices)
find_package(Jsoncpp)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS roscpp rexos_statemachine rexos_modbus rexos_utilities rexos_datatypes rexos_stewart_gough actionlib rexos_coordinates rexos_logger matrices
DEPENDS Jsoncpp)

include_directories(include ${catkin_INCLUDE_DIRS} )
add_executable(stewart_gough_node src/StewartGoughNode.cpp)
target_link_libraries(stewart_gough_node ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})