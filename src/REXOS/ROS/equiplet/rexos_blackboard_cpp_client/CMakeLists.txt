# blackboardCppClient #
cmake_minimum_required(VERSION 2.8.3)
project(rexos_blackboard_cpp_client)

## Find catkin and any catkin packages
find_package(catkin REQUIRED rexos_logger COMPONENTS rexos_utilities)
find_package(Boost)
find_package(Mongo)
find_package(OpenSSL)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_blackboard_cpp_client  rexos_logger
CATKIN_DEPENDS rexos_utilities rexos_logger
DEPENDS Boost Mongo OpenSSL)

SET ( CMAKE_CXX_FLAGS "-std=c++0x" )

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${MONGO_INCLUDE_DIRS} ${OPENSSL_INCLUDE_DIR})
add_library(rexos_blackboard_cpp_client ${sources})
target_link_libraries(rexos_blackboard_cpp_client ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${MONGO_LIBRARIES} ${OPENSSL_LIBRARIES})

