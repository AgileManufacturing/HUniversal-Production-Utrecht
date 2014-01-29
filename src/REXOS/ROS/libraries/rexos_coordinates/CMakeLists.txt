cmake_minimum_required(VERSION 2.8.3)
project(rexos_coordinates)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS vectors rexos_knowledge_database libjson)
find_package(MYSQLCONNECTORCPP)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_coordinates
CATKIN_DEPENDS vectors rexos_knowledge_database libjson
DEPENDS MYSQLCONNECTORCPP)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_coordinates ${sources})
target_link_libraries(rexos_coordinates ${catkin_LIBRARIES})
