cmake_minimum_required(VERSION 2.8.3)
project(roscpp_benchmark)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs std_srvs actionlib actionlib_msgs genmsg)

file(GLOB_RECURSE actions RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/action" "*.action")
add_action_files(
DIRECTORY action
FILES ${actions} )

generate_messages (
	DEPENDENCIES actionlib_msgs std_msgs
)


## Declare a catkin package
catkin_package(
INCLUDE_DIRS 
LIBRARIES  
CATKIN_DEPENDS roscpp std_msgs std_srvs actionlib actionlib_msgs
DEPENDS)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(node_a "src/nodeA.cpp")
add_dependencies(node_a roscpp_benchmark_gencpp)
target_link_libraries(node_a ${catkin_LIBRARIES})
add_executable(node_b "src/nodeB.cpp")
add_dependencies(node_b roscpp_benchmark_gencpp)
target_link_libraries(node_b ${catkin_LIBRARIES})

