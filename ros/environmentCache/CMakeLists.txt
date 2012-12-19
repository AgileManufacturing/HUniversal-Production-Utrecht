##############################################################
# CMakeLists.txt for a ROS package within the REXOS
# build system.
#
# Created by Dennis Koole, 10-11-2012
#
# 10-11-2012 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("environmentCache" true true)

# Add executables as documented by ROS
rosbuild_add_executable(EnvironmentCache src/EnvironmentCache.cpp)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()