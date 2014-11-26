cmake_minimum_required(VERSION 2.8.3)
project(part_locator_node)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation vision_node vectors matrices rexos_utilities environment_cache rexos_logger rexos_module)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_logger  
CATKIN_DEPENDS message_runtime roscpp std_msgs vision_node vectors matrices environment_cache rexos_logger rexos_module
DEPENDS OpenCV)

SET ( CMAKE_CXX_FLAGS "-std=c++0x" )

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(part_locator_node src/part_locator_node.cpp)
add_dependencies(part_locator_node part_locator_node_gencpp)
target_link_libraries(part_locator_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} rexos_utilities)
