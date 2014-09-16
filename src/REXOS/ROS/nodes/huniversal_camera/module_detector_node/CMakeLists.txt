cmake_minimum_required(VERSION 2.8.3)
project(module_detector_node)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation vision_node rexos_logger)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(Mongo)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_logger 
CATKIN_DEPENDS message_runtime roscpp std_msgs vision_node rexos_logger
DEPENDS OpenCV)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(module_detector_node src/module_detector_node.cpp)
add_dependencies(module_detector_node module_detector_node_gencpp)
target_link_libraries(module_detector_node ${catkin_LIBRARIES})
