cmake_minimum_required(VERSION 2.8.3)
project(rexos_utilities)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS)
find_package(Boost)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_utilities 
CATKIN_DEPENDS 
DEPENDS Boost)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_utilities ${sources})
target_link_libraries(rexos_utilities ${catkin_LIBRARIES} ${Boost_LIBRARIES})
