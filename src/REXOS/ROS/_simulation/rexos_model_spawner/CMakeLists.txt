cmake_minimum_required(VERSION 2.8.3)
project(rexos_model_spawner)



## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_knowledge_database rexos_logger rexos_datatypes rexos_zip gazebo_msgs collision_plugin)
find_package(Boost REQUIRED)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_model_spawner
CATKIN_DEPENDS rexos_knowledge_database rexos_logger rexos_datatypes gazebo_msgs collision_plugin
DEPENDS Boost)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_model_spawner ${sources})
target_link_libraries(rexos_model_spawner ${Boost_LIBRARIES} ${catkin_LIBRARIES})
