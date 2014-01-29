cmake_minimum_required(VERSION 2.8.3)
project(follow_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS crate_locator_node delta_robot_node)



###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES follow_node
  CATKIN_DEPENDS crate_locator_node delta_robot_node
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(follow_node src/FollowNode.cpp)

## Add dependencies to the executable
add_dependencies(follow_node crate_locator_node delta_robot_node)

## Specify libraries to link a library or executable target against
target_link_libraries(follow_node
  ${catkin_LIBRARIES}
)