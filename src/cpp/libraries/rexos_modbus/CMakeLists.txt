cmake_minimum_required(VERSION 2.8.3)
project(rexos_modbus)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_utilities)
find_package(Boost)
find_package(Modbus)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_modbus 
CATKIN_DEPENDS rexos_utilities
DEPENDS Boost Modbus)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${MODBUS_INCLUDE_DIRS})
add_library(rexos_modbus ${sources})
target_link_libraries(rexos_modbus ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${MODBUS_LIBRARIES})
