cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sensor_manager_plugin)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs rexos_io)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES 
CATKIN_DEPENDS roscpp rexos_io
DEPENDS Boost gazebo
)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_library(sensor_manager_plugin SHARED ${sources})

target_link_libraries(sensor_manager_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
