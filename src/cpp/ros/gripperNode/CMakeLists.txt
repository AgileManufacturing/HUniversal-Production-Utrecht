##############################################################
# CMakeLists.txt for a ROS package within the rexos
# build system.
#
# Created by Koen Braham
#
# 05-11-2012 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("gripperNode" false true)

# Add executables as documented by ROS
rosbuild_add_executable(gripperNode src/GripperNode.cpp)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake

include_directories(${MODBUS_INCLUDE_DIRS} ${LIB_MODBUS_CONTROLLER_INCLUDE} ${LIB_INPUT_OUTPUT_INCLUDE} ${LIB_UTILITIES_INCLUDE})

target_link_libraries(gripperNode ${MODBUS_LIBRARIES} inputOutput utilities)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()

