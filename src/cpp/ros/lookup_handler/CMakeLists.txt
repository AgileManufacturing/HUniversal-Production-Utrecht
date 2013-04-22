cmake_minimum_required(VERSION 2.8.3)
project(lookup_handler)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp environment_communication_msgs environment_cache std_msgs message_generation )
find_package(Log4cxx)

file(GLOB_RECURSE msgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${msgs} )
  	
file(GLOB_RECURSE srvs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "*.srv")
add_service_files(
DIRECTORY srv
FILES ${srvs} )

generate_messages (
	DEPENDENCIES std_msgs environment_communication_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS roscpp environment_communication_msgs environment_cache std_msgs 
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
add_executable(lookup_handler src/LookupHandler.cpp)
target_link_libraries(lookup_handler ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})
add_dependencies(lookup_handler lookup_handler_gencpp)
