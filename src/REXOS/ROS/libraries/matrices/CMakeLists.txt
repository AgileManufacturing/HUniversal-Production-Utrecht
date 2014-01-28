cmake_minimum_required(VERSION 2.8.3)
project(matrices)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS vectors)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES matrices
CATKIN_DEPENDS vectors
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(matrices ${sources})
