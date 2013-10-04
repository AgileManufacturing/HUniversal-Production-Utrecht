cmake_minimum_required(VERSION 2.8.3)
project(camera_control_node)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp camera_node camera rexos_statemachine)


## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES  
CATKIN_DEPENDS roscpp camera_node camera rexos_statemachine
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${camera_node_INCLUDE_DIRS})
add_executable(camera_control_node src/camera_control_node.cpp)
add_dependencies(camera_control_node camera_node)
target_link_libraries(camera_control_node ${catkin_LIBRARIES})
