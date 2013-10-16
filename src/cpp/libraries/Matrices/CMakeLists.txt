cmake_minimum_required(VERSION 2.8.3)
project(Matrices)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS Vectors)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES Matrices
CATKIN_DEPENDS Vectors
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(Matrices ${sources})