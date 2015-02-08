cmake_minimum_required(VERSION 2.8.3)
project(hardcoded_mover_node)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rexos_datatypes rexos_logger vectors rexos_module)
find_package(Mongo)
find_package(Jsoncpp)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES 
CATKIN_DEPENDS roscpp rexos_datatypes rexos_logger vectors rexos_module
DEPENDS Jsoncpp)

SET ( CMAKE_CXX_FLAGS "-std=c++0x" )

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(hardcoded_mover_node ${sources})

## Specify libraries to link a library or executable target against
target_link_libraries(hardcoded_mover_node ${catkin_LIBRARIES} rexos_utilities)
