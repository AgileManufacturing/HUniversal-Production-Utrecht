cmake_minimum_required(VERSION 2.8.3)
project(rexos_motor)



## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_modbus rexos_utilities rexos_datatypes )



## Declare a catkin package
catkin_package(INCLUDE_DIRS include LIBRARIES rexos_motor CATKIN_DEPENDS rexos_modbus   rexos_utilities rexos_datatypes )


file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_motor ${sources})
target_link_libraries(rexos_motor ${catkin_LIBRARIES})






#crexos_add_library(rexos_motor false "rexos_modbus;rexos_datatypes" "Boost")
#crexos_catkin_package()