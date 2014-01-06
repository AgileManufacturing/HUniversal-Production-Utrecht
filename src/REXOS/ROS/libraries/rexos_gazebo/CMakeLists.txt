cmake_minimum_required(VERSION 2.8.3)
project(rexos_gazebo)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp message_generation gazebo_ros)
find_package(gazebo REQUIRED)


## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_gazebo
CATKIN_DEPENDS roscpp
DEPENDS 
gazebo_ros
)

link_directories(${GAZEBO_LIBRARY_DIRS})

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_LIBRARIES})
add_library(rexos_gazebo ${sources})

target_link_libraries(rexos_gazebo ${catkin_LIBRARIES} ${Boost_LIBRARIES})



