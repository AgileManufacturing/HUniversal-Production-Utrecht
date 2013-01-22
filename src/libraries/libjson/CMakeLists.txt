cmake_minimum_required(VERSION 2.8.3)
project(libJson)

find_package(catkin REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES koen
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
crexos_add_library(libJson true)
