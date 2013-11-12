cmake_minimum_required(VERSION 2.8.3)
project(gripper_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_statemachine actionlib rexos_modbus rexos_std_msgs rexos_std_srvs rexos_utilities rexos_datatypes rexos_gripper libjson)
find_package(Log4cxx)

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
CATKIN_DEPENDS actionlib rexos_statemachine rexos_modbus rexos_std_msgs rexos_std_srvs rexos_utilities rexos_datatypes rexos_gripper libjson roscpp std_msgs
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
add_executable(gripper_node src/GripperNode.cpp)


target_link_libraries(gripper_node ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})
add_dependencies(gripper_node gripper_node_gencpp)

add_definitions("-std=c++0x")