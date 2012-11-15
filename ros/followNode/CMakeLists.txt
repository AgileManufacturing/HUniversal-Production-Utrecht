##############################################################
# CMakeLists.txt for a ROS package within the REXOS build system.
#
# Created by Koen Braham & Daan Veltman, 2012-10-18
#
# 2012-10-30 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("followNode" false false)

# Add executables as documented by ROS
rosbuild_add_executable(followNode src/FollowNode.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(followNode deltaRobotNode crateLocatorNode)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE} ${LIB_VISION_INCLUDE} ${NODE_CRATE_LOCATOR} ${NODE_DELTA_ROBOT})

# Link target to our libraries
target_link_libraries(followNode vision dataTypes)

# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()