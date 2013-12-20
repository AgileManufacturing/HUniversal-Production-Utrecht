cmake_minimum_required(VERSION 2.8.3)
project(arm_control_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs gazebo_ros simulation_node)

file(GLOB_RECURSE msgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${msgs} )

find_package(gazebo REQUIRED)
  	
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
CATKIN_DEPENDS roscpp simulation_node
DEPENDS 
gazebo_ros
)


link_directories(${GAZEBO_LIBRARY_DIRS})

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES})
add_executable(arm_control_node ${sources})
target_link_libraries(arm_control_node ${catkin_LIBRARIES})
add_dependencies(arm_control_node arm_control_node_gencpp)
