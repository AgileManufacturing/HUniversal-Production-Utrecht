cmake_minimum_required(VERSION 2.8.3)
project(rexos_configuration)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS)
find_package(Jsoncpp)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_configuration
CATKIN_DEPENDS 
DEPENDS Jsoncpp)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_configuration ${sources})
target_link_libraries(rexos_configuration ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
