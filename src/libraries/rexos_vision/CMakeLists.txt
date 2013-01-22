cmake_minimum_required(VERSION 2.8.3)
project(rexos_vision)

find_package(catkin REQUIRED rexos_datatypes)


catkin_package(
#  INCLUDE_DIRS include
   LIBRARIES rexos_vision
   CATKIN_DEPENDS rexos_datatypes
#  DEPENDS system_lib
)
crexos_add_library(rexos_vision false)

include_directories(${ZBAR_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${rexos_datatypes_INCLUDE_DIRS})
target_link_libraries(rexos_vision ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ZBAR_LIBRARIES}  ${rexos_datatypes_LIBRARIES})

