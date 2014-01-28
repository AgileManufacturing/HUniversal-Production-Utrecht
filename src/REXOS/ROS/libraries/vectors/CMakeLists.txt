cmake_minimum_required(VERSION 2.8.3)
project(vectors)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES vectors
CATKIN_DEPENDS 
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(vectors ${sources})
target_link_libraries(vectors ${catkin_LIBRARIES})
