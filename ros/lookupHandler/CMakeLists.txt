##############################################################
# CMakeLists.txt for a ROS package within the REXOS
# build system.
#
# Created by Arjan Groenewegen, 10-11-2012
#
# 10-11-2012 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("lookupHandler" true true)

# Add executables as documented by ROS
rosbuild_add_executable(lookupHandler src/LookupHandler.cpp)

# Link target to our libraries
target_link_libraries(lookupHandler json)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_JSON_INCLUDE})

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()