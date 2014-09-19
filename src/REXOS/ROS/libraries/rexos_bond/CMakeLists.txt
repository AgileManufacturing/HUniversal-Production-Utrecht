cmake_minimum_required(VERSION 2.8.3)
project(rexos_bond)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED bond roscpp smclib rexos_logger)

include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

if(WIN32)
  set(UUID_LIB "Rpcrt4")
else()
  set(UUID_LIB "uuid")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${UUID_LIB} rexos_logger
  CATKIN_DEPENDS bond roscpp smclib rexos_logger
  DEPENDS Boost
)

add_library(${PROJECT_NAME}
  src/Timeout.cpp
  src/Bond.cpp
  src/BondSM_sm.cpp
)
target_link_libraries(${PROJECT_NAME} ${UUID_LIB} ${catkin_LIBRARIES} ${BOOST_LIBRARIES})

if(catkin_EXPORTED_TARGETS)
  add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
endif()

# install stuff
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
