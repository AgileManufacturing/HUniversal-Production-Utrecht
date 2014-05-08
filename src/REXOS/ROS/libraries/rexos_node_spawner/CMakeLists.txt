cmake_minimum_required(VERSION 2.8.3)
project(rexos_node_spawner)



## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS rexos_knowledge_database)
find_package(Boost REQUIRED)
find_package(LIBZIP REQUIRED)
find_package(ZLIB REQUIRED)

INCLUDE_DIRECTORIES(${ZLIB_INCLUDE_DIR})
INCLUDE (FindPkgConfig)
if (PKG_CONFIG_FOUND)
        PKG_CHECK_MODULES(UNZIP minizip)
endif (PKG_CONFIG_FOUND)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_node_spawner
CATKIN_DEPENDS rexos_knowledge_database
DEPENDS Boost LIBZIP)

MESSAGE("${LIBZIP_LIBRARIES}")
MESSAGE("${ZLIB_LIBRARIES}")
MESSAGE("${UNZIP_LIBRARIES}")

MESSAGE("${LIBZIP_INCLUDE_DIRS}")
MESSAGE("${ZLIB_INCLUDE_DIRS}")
MESSAGE("${UNZIP_INCLUDE_DIRS}")

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_node_spawner ${sources})
target_link_libraries(rexos_node_spawner ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${LIBZIP_LIBRARIES} /usr/local/lib/libzip.so)
