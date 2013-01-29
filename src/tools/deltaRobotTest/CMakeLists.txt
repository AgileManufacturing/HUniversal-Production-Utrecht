##############################################################
# CMakeLists.txt for a ROS package within the rexos
# build system.
#
# Created by Dick van der Steen <dickvandersteen@gmail.com>
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("deltaRobotTest" false false)

# Add executables as documented by ROS
rosbuild_add_executable(DeltaRobotTest src/DeltaRobotTest.cpp)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories("${NODE_DELTA_ROBOT}")

# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()
