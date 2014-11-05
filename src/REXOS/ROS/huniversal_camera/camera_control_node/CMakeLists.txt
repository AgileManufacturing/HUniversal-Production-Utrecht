cmake_minimum_required(VERSION 2.8.3)
project(camera_control_node)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp vision_node camera_calibration_node camera rexos_statemachine rexos_knowledge_database rexos_logger rexos_datatypes)
find_package(MYSQLCONNECTORCPP)
find_package(Jsoncpp)


## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES  
CATKIN_DEPENDS roscpp vision_node camera camera_calibration_node rexos_statemachine rexos_knowledge_database rexos_logger rexos_datatypes
DEPENDS Jsoncpp)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")

include_directories(include ${catkin_INCLUDE_DIRS} ${camera_node_INCLUDE_DIRS} ${camera_calibration_node_INCLUDE_DIRS})
add_executable(camera_control_node src/camera_control_node.cpp)
add_dependencies(camera_control_node camera_node camera_calibration_node MYSQLCONNECTORCPP)
target_link_libraries(camera_control_node ${catkin_LIBRARIES} ${MYSQLCONNECTORCPP_LIBRARIES} ${JSONCPP_LIBRARIES})
