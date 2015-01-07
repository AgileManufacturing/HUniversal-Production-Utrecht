cmake_minimum_required(VERSION 2.8.3)
project(image_stream_node)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_utilities image_transport rexos_logger)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(cv_bridge REQUIRED COMPONENTS)

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
LIBRARIES rexos_logger 
CATKIN_DEPENDS message_runtime roscpp std_msgs camera rexos_utilities image_transport rexos_logger
DEPENDS cv_bridge)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(image_stream_node src/image_stream_node.cpp)
add_dependencies(image_stream_node image_stream_node_gencpp)
target_link_libraries(image_stream_node ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES})
