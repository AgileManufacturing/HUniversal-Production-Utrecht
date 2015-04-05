cmake_minimum_required(VERSION 2.8.3)
project(rexos_node_spawner)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_knowledge_database rexos_logger rexos_datatypes rexos_zip)
find_package(Boost REQUIRED)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_node_spawner
CATKIN_DEPENDS rexos_knowledge_database rexos_logger rexos_datatypes rexos_zip
DEPENDS Boost LIBZIP)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_node_spawner ${sources})
target_link_libraries(rexos_node_spawner ${Boost_LIBRARIES} ${catkin_LIBRARIES})
