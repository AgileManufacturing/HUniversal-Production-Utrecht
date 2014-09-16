#cmake_minimum_required(VERSION 2.8.3)
#project(keyboard_control_node)
#
### Find catkin macros and libraries
#find_package(catkin REQUIRED COMPONENTS roscpp rexos_blackboard_cpp_client rexos_datatypes rexos_utilities)
#find_package(Mongo)
#find_package(Log4cxx)
#
####################################################
### Declare things to be passed to other projects ##
####################################################
#
### Declare a catkin package
#catkin_package(
#INCLUDE_DIRS include
#LIBRARIES 
#CATKIN_DEPENDS roscpp rexos_blackboard_cpp_client rexos_datatypes rexos_utilities
#DEPENDS)
#
############
### Build ##
############
#
#SET ( CMAKE_CXX_FLAGS "-std=c++0x" )
#
##${Boost_INCLUDE_DIRS})
#
#file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
#include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
#add_executable(keyboard_control_node ${sources})
#
### Specify libraries to link a library or executable target against
#target_link_libraries(keyboard_control_node ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES} rexos_utilities)
#