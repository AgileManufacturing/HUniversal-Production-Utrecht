cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gripper_plugin)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs rexos_io rexos_logger)
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
CATKIN_DEPENDS roscpp rexos_io rexos_logger
DEPENDS Boost gazebo
)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_library(gripper_plugin SHARED ${sources})

target_link_libraries(gripper_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
