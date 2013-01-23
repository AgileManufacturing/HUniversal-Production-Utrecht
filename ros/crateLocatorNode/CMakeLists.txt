##############################################################
# CMakeLists.txt for a ROS package within the REXOS build system.
#
# Created by Koen Braham & Daan Veltman, 2012-10-18
#
# 2012-10-18 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("crateLocatorNode" true true)

# Add executables as documented by ROS
rosbuild_add_executable(crateLocatorNode src/CrateLocatorNode.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(crateLocatorNode vision)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE} ${LIB_VISION_INCLUDE} ${LIB_JSON_INCLUDE})

# Link target to our libraries
target_link_libraries(crateLocatorNode vision dataTypes json)

# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()