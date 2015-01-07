cmake_minimum_required(VERSION 2.8.3)
project(part_follow_node)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rexos_blackboard_cpp_client rexos_datatypes rexos_utilities rexos_logger)
find_package(Mongo)

###################################################
## Declare things to be passed to other projects ##
###################################################

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_logger
CATKIN_DEPENDS roscpp rexos_blackboard_cpp_client rexos_datatypes rexos_utilities rexos_logger
DEPENDS)

###########
## Build ##
###########

SET ( CMAKE_CXX_FLAGS "-std=c++0x" )

#${Boost_INCLUDE_DIRS})

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(part_follow_node ${sources})

## Specify libraries to link a library or executable target against
target_link_libraries(part_follow_node ${catkin_LIBRARIES} rexos_utilities)
