cmake_minimum_required(VERSION 2.8.3)
project(stewart_gough_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_statemachine rexos_modbus rexos_std_msgs rexos_std_srvs rexos_utilities rexos_datatypes rexos_stewart_gough actionlib rexos_coordinates rexos_logger)
find_package(Log4cxx)
find_package(Jsoncpp)

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
CATKIN_DEPENDS actionlib rexos_statemachine rexos_modbus rexos_std_msgs rexos_std_srvs rexos_utilities rexos_stewart_gough rexos_datatypes roscpp std_msgs rexos_coordinates rexos_logger
DEPENDS Jsoncpp)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
add_executable(stewart_gough_node src/StewartGoughNode.cpp)
target_link_libraries(stewart_gough_node ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES} ${JSONCPP_LIBRARIES})
add_dependencies(stewart_gough_node stewart_gough_node_gencpp)

add_definitions("-std=c++0x")
