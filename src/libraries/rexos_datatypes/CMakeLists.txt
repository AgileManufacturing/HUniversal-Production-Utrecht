cmake_minimum_required(VERSION 2.8.3)
project(rexos_datatypes)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS)
find_package(Boost)
find_package(OpenCV)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_datatypes 
CATKIN_DEPENDS 
DEPENDS Boost OpenCV)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_datatypes ${sources})
target_link_libraries(rexos_datatypes ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})



