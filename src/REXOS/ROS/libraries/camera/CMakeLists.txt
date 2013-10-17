cmake_minimum_required(VERSION 2.8.3)
project(camera)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS)
find_package(OpenCV REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS)
find_package(Unicap REQUIRED COMPONENTS)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include ${UNICAP_INCLUDE_DIRS}
LIBRARIES camera
CATKIN_DEPENDS 
DEPENDS OpenCV Boost Unicap)

MESSAGE(hoi: ${UNICAP_INCLUDE_DIRS})

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${UNICAP_INCLUDE_DIRS})
add_library(camera ${sources})
target_link_libraries(camera ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${UNICAP_LIBRARIES})
