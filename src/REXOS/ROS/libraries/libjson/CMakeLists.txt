cmake_minimum_required(VERSION 2.8.3)
project(libjson)


## Find catkin and any catkin packages
find_package(catkin REQUIRED rexos_logger)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES libjson rexos_logger
CATKIN_DEPENDS rexos_logger
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(libjson ${sources})
target_link_libraries(libjson ${catkin_LIBRARIES})

