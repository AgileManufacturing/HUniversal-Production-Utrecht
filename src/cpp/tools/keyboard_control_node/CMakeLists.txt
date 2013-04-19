cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control_node)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS delta_robot_node)



###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES keyboard_control_node
  CATKIN_DEPENDS delta_robot_node
)

###########
## Build ##
###########

## Specify additional locations of header files
 include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp library
# add_library(keyboard_control_node
#   src/${PROJECT_NAME}/keyboard_control_node.cpp
# )

## Declare a cpp executable
add_executable(keyboard_control_node src/KeyBoardControlNode.cpp)

## Add dependencies to the executable
 add_dependencies(keyboard_control_node delta_robot_node)

## Specify libraries to link a library or executable target against
 target_link_libraries(keyboard_control_node
   ${catkin_LIBRARIES}
 )
