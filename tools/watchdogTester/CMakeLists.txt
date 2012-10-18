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
rexos_rospack_init("watchdogTester" false false)

# Add executables as documented by ROS
rosbuild_add_executable(watchdogTester src/watchdogTester.cpp)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake

include_directories(${MODBUS_INCLUDE_DIRS} ${LIB_MODBUS_CONTROLLER_INCLUDE} ${LIB_INPUT_OUTPUT_INCLUDE})

target_link_libraries(watchdogTester ${MODBUS_LIBRARIES} inputOutput)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()
