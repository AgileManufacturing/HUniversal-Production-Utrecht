cmake_minimum_required(VERSION 2.8.3)
project(rexos_datatypes)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES rexos_datatypes
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
crexos_add_library(rexos_datatypes false)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(rexos_datatypes ${Boost_LIBRARIES} ${OpenCV_LIBS})






