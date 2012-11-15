##############################################################
# CMakeLists.txt for a ROS package within the rexos
# build system.
#
# Created by Arjan Groenewegen <arjanhgroen@gmail.com>
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("rosMast" true true)

rosbuild_add_library(rosMast src/StateMachine.cpp)

# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()