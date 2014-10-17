cmake_minimum_required(VERSION 2.8.3)
project(dummy_module_b)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_knowledge_database rexos_statemachine rexos_logger)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include
LIBRARIES rexos_logger  
CATKIN_DEPENDS message_runtime roscpp std_msgs rexos_knowledge_database rexos_statemachine rexos_logger
DEPENDS )

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(dummy_module_b src/dummy_module_b.cpp)
add_dependencies(dummy_module_b dummy_module_b_gencpp)
target_link_libraries(dummy_module_b ${catkin_LIBRARIES})
