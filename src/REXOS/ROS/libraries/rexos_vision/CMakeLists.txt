cmake_minimum_required(VERSION 2.8.3)
project(rexos_vision)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_datatypes)
find_package(Boost)
find_package(ZBAR)
find_package(OpenCV)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_vision 
CATKIN_DEPENDS rexos_datatypes
DEPENDS Boost ZBAR OpenCV)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_vision ${sources})
target_link_libraries(rexos_vision ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${ZBAR_LIBRARIES} ${OpenCV_LIBRARIES})