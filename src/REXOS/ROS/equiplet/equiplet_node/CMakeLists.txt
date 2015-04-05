cmake_minimum_required(VERSION 2.8.3)
project(equiplet_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs actionlib_msgs actionlib node_spawner_node model_spawner_node message_generation rexos_configuration rexos_statemachine rexos_datatypes rexos_blackboard_cpp_client rexos_logger rexos_module)
find_package(Mongo)
find_package(Jsoncpp)

file(GLOB_RECURSE actions RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/action" "*.action")
add_action_files(
DIRECTORY action
FILES ${actions} )

file(GLOB_RECURSE srvs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "*.srv")
add_service_files(
DIRECTORY srv
FILES ${srvs} )

generate_messages (
	DEPENDENCIES actionlib_msgs std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS roscpp std_msgs node_spawner_node model_spawner_node rexos_configuration rexos_statemachine rexos_datatypes rexos_blackboard_cpp_client rexos_logger rexos_module
DEPENDS Jsoncpp)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
add_executable(equiplet_node ${sources})

target_link_libraries(equiplet_node ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES} ${JSONCPP_LIBRARIES} ${CMAKE_DL_LIBS})
add_dependencies(equiplet_node equiplet_node_gencpp)

