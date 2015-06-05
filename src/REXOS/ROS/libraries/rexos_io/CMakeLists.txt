cmake_minimum_required(VERSION 2.8.3)
project(rexos_io)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs message_generation rexos_utilities rexos_datatypes rexos_logger)
find_package(Modbus REQUIRED)

file(GLOB_RECURSE srvs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/srv" "*.srv")
add_service_files(
DIRECTORY srv
FILES ${srvs} )

generate_messages (
	DEPENDENCIES std_msgs
)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES rexos_io 
CATKIN_DEPENDS message_runtime rexos_utilities rexos_datatypes rexos_logger
DEPENDS modbus
)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(rexos_io ${sources})
add_dependencies(rexos_io rexos_io_gencpp )
target_link_libraries(rexos_io ${catkin_LIBRARIES} ${MODBUS_LIBRARIES})
