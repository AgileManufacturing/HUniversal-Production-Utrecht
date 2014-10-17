cmake_minimum_required(VERSION 2.8.3)
project(environment_cache)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation rexos_logger rexos_environment_cache)
find_package(Jsoncpp)

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
LIBRARIES rexos_logger 
CATKIN_DEPENDS roscpp std_msgs rexos_logger rexos_environment_cache
DEPENDS Jsoncpp)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(environment_cache src/EnvironmentCache.cpp)
target_link_libraries(environment_cache ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
add_dependencies(environment_cache environment_cache_gencpp)

add_executable(environment_cache_test src/EnvironmentCacheTest.cpp)
target_link_libraries(environment_cache_test ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})
add_dependencies(environment_cache_test environment_cache_gencpp)
