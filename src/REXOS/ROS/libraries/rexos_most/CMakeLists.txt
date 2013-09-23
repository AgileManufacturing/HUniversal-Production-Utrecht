cmake_minimum_required(VERSION 2.8.3)
project(rexos_most)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rexos_statemachine std_msgs genmsg message_generation)
find_package(Mongo)

generate_messages (
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_most 
CATKIN_DEPENDS rexos_statemachine
DEPENDS Boost)

file(GLOB_RECURSE sources "src/**.c" "src/**.cpp")
include_directories(include ${catkin_INCLUDE_DIRS} ${MONGO_INCLUDE_DIRS})
message(STATUS "sources: ${sources}")
add_library(rexos_most ${sources})

target_link_libraries(rexos_most ${catkin_LIBRARIES} ${MONGO_LIBRARIES})
add_dependencies(rexos_most rexos_most_gencpp)

add_definitions("-std=c++0x")