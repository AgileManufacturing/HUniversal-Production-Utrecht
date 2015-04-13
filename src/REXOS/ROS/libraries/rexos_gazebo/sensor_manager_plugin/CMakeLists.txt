cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sensor_manager_plugin)

## Find catkin and any catkin packages
find_package(catkin REQUIRED std_msgs message_generation)
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

file(GLOB_RECURSE srvs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "*.srv")
add_service_files(
DIRECTORY srv
FILES ${srvs} )

generate_messages (
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES 
CATKIN_DEPENDS message_runtime roscpp std_msgs 
DEPENDS Boost gazebo
)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
add_library(sensor_manager_plugin SHARED ${sources})

add_dependencies(sensor_manager_plugin sensor_manager_plugin_gencpp )
target_link_libraries(sensor_manager_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
