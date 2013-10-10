cmake_minimum_required(VERSION 2.8.3)
project(dummy_module_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_statemachine rexos_std_msgs rexos_std_srvs libjson rexos_utilities)
find_package(Log4cxx)

generate_messages (
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS rexos_statemachine rexos_std_msgs rexos_std_srvs libjson roscpp std_msgs rexos_utilities
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
add_executable(dummy_module_node src/DummyModuleNode.cpp)

target_link_libraries(dummy_module_node ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})
add_dependencies(dummy_module_node dummy_module_node_gencpp)

add_definitions("-std=c++0x")