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
rexos_rospack_init("equipletNode" true true)

# Add executables as documented by ROS
file(GLOB sources src/*.cpp)
rosbuild_add_executable(EquipletNode ${sources})

include_directories(${LIB_MAST_INCLUDE})

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_MAST_INCLUDE})

# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()