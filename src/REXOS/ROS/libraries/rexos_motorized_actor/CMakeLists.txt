cmake_minimum_required(VERSION 2.8.3)
project(rexos_motorized_actor)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_datatypes rexos_io rexos_motor rexos_sensor rexos_logger rexos_utilities)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_motorized_actor 
CATKIN_DEPENDS rexos_datatypes rexos_io rexos_motor rexos_sensor rexos_logger rexos_utilities
DEPENDS Boost)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_motorized_actor ${sources})

target_link_libraries(rexos_motorized_actor ${catkin_LIBRARIES})



