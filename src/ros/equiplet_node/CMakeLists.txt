cmake_minimum_required(VERSION 2.8.3)
project(equiplet_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs rexos_mast lookup_handler rexos_std_msgs rexos_std_srvs rexos_blackboard_cpp_client Libjson environment_communication_msgs)
find_package(Mongo)


## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS roscpp std_msgs rexos_mast lookup_handler rexos_std_msgs rexos_std_srvs rexos_blackboard_cpp_client Libjson environment_communication_msgs
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(equiplet_node src/EquipletNode.cpp)


target_link_libraries(equiplet_node ${catkin_LIBRARIES})



