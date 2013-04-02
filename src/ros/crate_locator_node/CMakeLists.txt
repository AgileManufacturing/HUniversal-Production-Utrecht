cmake_minimum_required(VERSION 2.8.3)
project(crate_locator_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation image_transport cv_bridge sensor_msgs rexos_datatypes rexos_utilities rexos_vision)
find_package(Log4cxx)
find_package(OpenCV)

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
CATKIN_DEPENDS roscpp image_transport cv_bridge sensor_msgs rexos_datatypes rexos_utilities rexos_vision
DEPENDS OpenCV)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
add_executable(crate_locator_node src/CrateLocatorNode.cpp)
target_link_libraries(crate_locator_node ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})
add_dependencies(crate_locator_node crate_locator_node_gencpp)
