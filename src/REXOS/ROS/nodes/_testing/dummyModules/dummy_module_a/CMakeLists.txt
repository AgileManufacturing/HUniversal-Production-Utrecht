cmake_minimum_required(VERSION 2.8.3)
project(dummy_module_a)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_knowledge_database rexos_statemachine)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES  
CATKIN_DEPENDS message_runtime roscpp std_msgs rexos_knowledge_database rexos_statemachine
DEPENDS )

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(dummy_module_a src/dummy_module_a.cpp)
add_dependencies(dummy_module_a dummy_module_a_gencpp)
target_link_libraries(dummy_module_a ${catkin_LIBRARIES})
