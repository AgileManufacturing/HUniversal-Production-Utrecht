cmake_minimum_required(VERSION 2.8.3)
project(rexos_stewart_gough)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_datatypes rexos_utilities vectors matrices rexos_logger rexos_motorized_actor)
find_package(Boost)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_stewart_gough 
CATKIN_DEPENDS rexos_datatypes rexos_utilities vectors matrices rexos_logger rexos_motorized_actor
DEPENDS Boost)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_stewart_gough ${sources})

target_link_libraries(rexos_stewart_gough ${catkin_LIBRARIES} ${Boost_LIBRARIES})



