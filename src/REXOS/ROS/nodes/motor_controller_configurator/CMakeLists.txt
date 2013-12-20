cmake_minimum_required(VERSION 2.8.3)
project(motor_controller_configurator)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rexos_utilities rexos_modbus rexos_motor)
find_package(Log4cxx)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS include 
LIBRARIES  
CATKIN_DEPENDS roscpp rexos_utilities rexos_modbus rexos_motor
DEPENDS)

file(GLOB_RECURSE sources "src" "*.cpp" "*.c")
include_directories(include ${catkin_INCLUDE_DIRS} ${LOG4CXX_INCLUDE_DIR})
add_executable(motor_controller_configurator src/MotorControllerConfigurator.cpp)
target_link_libraries(motor_controller_configurator ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})
add_dependencies(motor_controller_configurator rexos_utilities)
