cmake_minimum_required(VERSION 2.8.3)
project(rexos_std_srvs)
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation )
  	
file(GLOB_RECURSE srvs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "*.srv")
add_service_files(
DIRECTORY srv
FILES ${srvs} )

generate_messages (
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
LIBRARIES  
CATKIN_DEPENDS rexos_std_msgs
DEPENDS)

