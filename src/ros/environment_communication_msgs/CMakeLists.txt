cmake_minimum_required(VERSION 2.8.3)
project(environment_communication_msgs)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation )

file(GLOB_RECURSE msgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${msgs} )
  	
generate_messages (
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS 
LIBRARIES  
CATKIN_DEPENDS std_msgs
DEPENDS)

