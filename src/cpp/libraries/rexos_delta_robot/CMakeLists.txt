cmake_minimum_required(VERSION 2.8.3)
project(rexos_delta_robot)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_datatypes rexos_modbus rexos_motor rexos_utilities)
find_package(Boost)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_delta_robot 
CATKIN_DEPENDS rexos_datatypes rexos_modbus rexos_motor rexos_utilities
DEPENDS Boost)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_delta_robot ${sources})

target_link_libraries(rexos_delta_robot ${catkin_LIBRARIES} ${Boost_LIBRARIES})



