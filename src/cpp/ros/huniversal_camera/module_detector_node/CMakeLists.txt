cmake_minimum_required(VERSION 2.8.3)
project(module_detector_node)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation qr_code_reader_node)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(Mongo)

file(GLOB_RECURSE msgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${msgs} )
  	
file(GLOB_RECURSE srvs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "*.srv")
add_service_files(
DIRECTORY srv
FILES ${srvs} )

generate_messages (
	DEPENDENCIES std_msgs
)


## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES  
CATKIN_DEPENDS message_runtime roscpp std_msgs qr_code_reader_node
DEPENDS OpenCV)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(module_detector_node src/module_detector_node.cpp)
add_dependencies(module_detector_node module_detector_node_gencpp)
target_link_libraries(module_detector_node ${catkin_LIBRARIES})
