cmake_minimum_required(VERSION 2.8.3)
project(model_spawner_node)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_model_spawner rexos_knowledge_database rexos_logger rexos_datatypes)

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
CATKIN_DEPENDS roscpp std_msgs rexos_model_spawner rexos_knowledge_database rexos_logger rexos_datatypes
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(model_spawner_node ${sources})
add_dependencies(model_spawner_node model_spawner_node_gencpp)
target_link_libraries(model_spawner_node ${catkin_LIBRARIES})

