cmake_minimum_required(VERSION 2.8.3)
project(parts_reciever_node)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_utilities rexos_logger)

file(GLOB_RECURSE msgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${msgs} )

generate_messages (
	DEPENDENCIES std_msgs
)

catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_logger 
CATKIN_DEPENDS message_runtime roscpp std_msgs rexos_utilities rexos_logger
DEPENDS cv_bridge)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(parts_reciever_node ${sources})
add_dependencies(parts_reciever_node parts_reciever_node_gencpp )
target_link_libraries(parts_reciever_node ${catkin_LIBRARIES})