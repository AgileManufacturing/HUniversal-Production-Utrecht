##############################################################
# CMakeLists.txt for a ROS package within the rexos
# build system.
#
# Created by Dennis Koole <dennis.koole@gmail.com>
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("equipletNode" false false)

# Add executables as documented by ROS
file(GLOB sources src/*.cpp)
rosbuild_add_executable(EquipletNode ${sources})

# Link target to our libraries
target_link_libraries(EquipletNode utilities)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_MAST_INCLUDE} ${LIB_UTILITIES_INCLUDE})

# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()