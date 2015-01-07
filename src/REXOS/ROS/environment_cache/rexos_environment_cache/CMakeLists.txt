cmake_minimum_required(VERSION 2.8.3)
project(rexos_environment_cache)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rexos_logger)
find_package(Jsoncpp)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_environment_cache
CATKIN_DEPENDS roscpp rexos_logger
DEPENDS Jsoncpp)

include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})

add_library(rexos_environment_cache src/EnvironmentCache.cpp)
target_link_libraries(rexos_environment_cache ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})