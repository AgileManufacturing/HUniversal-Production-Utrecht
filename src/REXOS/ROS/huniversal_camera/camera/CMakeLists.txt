cmake_minimum_required(VERSION 2.8.3)
project(camera)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rexos_datatypes rexos_utilities image_transport rexos_logger)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(cv_bridge REQUIRED COMPONENTS)
find_package(Unicap REQUIRED COMPONENTS)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include ${UNICAP_INCLUDE_DIRS}
LIBRARIES camera
CATKIN_DEPENDS roscpp rexos_datatypes rexos_utilities image_transport rexos_logger
DEPENDS cv_bridge Boost Unicap )

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${UNICAP_INCLUDE_DIRS})
add_library(camera ${sources})
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${UNICAP_LIBRARIES})

