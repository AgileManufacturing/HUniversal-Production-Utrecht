cmake_minimum_required(VERSION 2.8.3)
project(rexos_statemachine)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg actionlib_msgs actionlib message_generation rexos_knowledge_database rexos_logger rexos_datatypes)
find_package(Mongo)

file(GLOB_RECURSE actions RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/action" "*.action")
add_action_files(
DIRECTORY action
FILES ${actions} )

file(GLOB_RECURSE services RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "*.srv")
add_service_files(
DIRECTORY srv
FILES ${services} )

generate_messages (
	DEPENDENCIES actionlib_msgs std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_statemachine rexos_logger
CATKIN_DEPENDS rexos_knowledge_database rexos_logger rexos_datatypes
DEPENDS Boost)

file(GLOB_RECURSE sources "src/**.c" "src/**.cpp")
include_directories(include ${catkin_INCLUDE_DIRS} ${MONGO_INCLUDE_DIRS})
add_library(rexos_statemachine ${sources})

target_link_libraries(rexos_statemachine ${catkin_LIBRARIES} ${MONGO_LIBRARIES})
add_dependencies(rexos_statemachine rexos_statemachine_gencpp)

add_definitions("-std=c++0x")
