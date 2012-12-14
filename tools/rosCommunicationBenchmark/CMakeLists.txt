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
rexos_rospack_init("rosCommunicationBenchmark" true true)

# Add executables as documented by ROS
rosbuild_add_executable(RosComBenchmarkServer src/RosComBenchmarkServer.cpp)
rosbuild_add_executable(RosComBenchmarkClient src/RosComBenchmarkClient.cpp)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories("${LIB_UTILITIES_INCLUDE}")

# Link target to our libraries
target_link_libraries(RosComBenchmarkServer utilities)
target_link_libraries(RosComBenchmarkClient utilities)

# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()
