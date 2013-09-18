cmake_minimum_required(VERSION 2.8.3)
project(delta_robot_test)

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
  LIBRARIES delta_robot_test
  CATKIN_DEPENDS delta_robot_node
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(delta_robot_test src/DeltaRobotTest.cpp)

## Add dependencies to the executable
add_dependencies(delta_robot_test delta_robot_node)

## Specify libraries to link a library or executable target against
target_link_libraries(delta_robot_test
  ${catkin_LIBRARIES}
)