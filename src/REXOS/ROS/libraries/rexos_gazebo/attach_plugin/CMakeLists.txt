cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(rexos_gazebo_attach_plugin)

# manually set find the boost library, as the gazebo plugin library requires boost v1.54 and ROS library requires boost v1.46
set(BOOST_ROOT ~/git/HUniversal-Production-Utrecht/)

## Find catkin and any catkin packages
find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

message("rexos_gazebo_attach_plugin boost path " ${Boost_INCLUDE_DIRS})

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES 
CATKIN_DEPENDS
DEPENDS Boost gazebo
)


#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo)
#endif()

SET(GCC_COVERAGE_COMPILE_FLAGS "-fBOOST_ENABLE_THREADS")

#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(rexos_gazebo_attach_plugin SHARED src/AttachPlugin.cpp)
target_link_libraries(rexos_gazebo_attach_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
