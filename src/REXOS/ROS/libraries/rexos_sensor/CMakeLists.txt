cmake_minimum_required(VERSION 2.8.3)
project(rexos_sensor)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_modbus rexos_utilities rexos_datatypes rexos_logger sensor_manager_plugin)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_sensor 
CATKIN_DEPENDS rexos_modbus rexos_utilities rexos_datatypes rexos_logger sensor_manager_plugin
DEPENDS
)


file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_sensor ${sources})
target_link_libraries(rexos_sensor ${catkin_LIBRARIES})
