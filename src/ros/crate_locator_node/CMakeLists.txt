cmake_minimum_required(VERSION 2.8.3)
project(crate_locator_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation image_transport cv_bridge opencv2 sensor_msgs rexos_datatypes rexos_utilities rexos_vision)

file(GLOB_RECURSE msgs "msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${msgs} )
  	
file(GLOB_RECURSE srvs "srv" "*.srv")
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
CATKIN_DEPENDS roscpp image_transport cv_bridge opencv2 sensor_msgs rexos_datatypes rexos_utilities rexos_vision
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(crate_locator_node src/CrateLocatorNode.cpp)
target_link_libraries(crate_locator_node ${catkin_LIBRARIES})
add_dependencies(crate_locator_node crate_locator_node_gencpp)
