cmake_minimum_required(VERSION 2.8.3)
project(fiducial_reader_node)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation camera_node camera image_transport rexos_vision)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(cv_bridge REQUIRED COMPONENTS)

file(GLOB_RECURSE msgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/msg" "*.msg")
add_message_files(
DIRECTORY msg
FILES ${msgs} )

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
CATKIN_DEPENDS roscpp std_msgs camera_node camera image_transport rexos_vision 
DEPENDS cv_bridge)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${camera_node_INCLUDE_DIRS})
add_executable(fiducial_reader_node src/fiducial_reader_node.cpp)
add_dependencies(fiducial_reader_node fiducial_reader_node_gencpp camera_node)
target_link_libraries(fiducial_reader_node ${catkin_LIBRARIES} ${cv_bridge_LIBRARIES})
