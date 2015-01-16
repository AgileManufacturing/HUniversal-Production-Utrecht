cmake_minimum_required(VERSION 2.8.3)
project(rexos_logger)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp)
## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_logger
CATKIN_DEPENDS roscpp
DEPENDS)
file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_logger ${sources})
target_link_libraries(rexos_logger ${catkin_LIBRARIES})
