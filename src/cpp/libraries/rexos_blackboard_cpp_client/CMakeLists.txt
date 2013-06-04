# blackboardCppClient #
cmake_minimum_required(VERSION 2.8.3)
project(rexos_blackboard_cpp_client)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_utilities)
find_package(Boost)
find_package(Mongo)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_blackboard_cpp_client 
CATKIN_DEPENDS rexos_utilities
DEPENDS Boost Mongo)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${MONGO_INCLUDE_DIRS})
add_library(rexos_blackboard_cpp_client ${sources})
target_link_libraries(rexos_blackboard_cpp_client ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${MONGO_LIBRARIES})

