cmake_minimum_required(VERSION 2.8.3)
project(rexos_module)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg actionlib_msgs actionlib message_generation rexos_logger rexos_datatypes rexos_knowledge_database rexos_coordinates rexos_bond rexos_statemachine)

file(GLOB_RECURSE actions RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/action" "*.action")
add_action_files(
DIRECTORY action
FILES ${actions} )

file(GLOB_RECURSE messages RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${messages} )

generate_messages (
	DEPENDENCIES actionlib_msgs std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_module
CATKIN_DEPENDS rexos_logger rexos_datatypes rexos_knowledge_database rexos_coordinates rexos_bond rexos_statemachine
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_module ${sources})
target_link_libraries(rexos_module ${catkin_LIBRARIES})
add_dependencies(rexos_module rexos_module_gencpp)
