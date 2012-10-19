##############################################################
# CMakeLists.txt for a ROS package within the low-cost-vision
# build system.
#
# Created by Pascal Muller <pascalmuller@gmail.com>, 09-02-2012
#
# 09-02-2012 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("deltaRobotNode" true true)

# Add executables as documented by ROS
rosbuild_add_executable(DeltaRobotNode src/DeltaRobotNode.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(DeltaRobotNode deltaRobot)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_DATATYPES_INCLUDE} ${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE} ${LIB_DELTAROBOT_INCLUDE} ${LIB_MOTOR_INCLUDE} ${LIB_MODBUS_CONTROLLER_INCLUDE})

# Link target to our libraries
target_link_libraries(DeltaRobotNode deltaRobot modbusController motor)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()