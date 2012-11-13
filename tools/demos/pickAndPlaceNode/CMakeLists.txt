##############################################################
# CMakeLists.txt for a ROS package within the REXOS build system.
#
# Created by Koen Braham & Daan Veltman, 2012-11-13
#
# 2012-11-13 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("pickAndPlaceNode" false false)

# Add executables as documented by ROS
rosbuild_add_executable(PickAndPlaceNode src/PickAndPlaceNode.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(PickAndPlaceNode deltaRobotNode crateLocatorNode)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE} ${LIB_VISION_INCLUDE} ${NODE_CRATE_LOCATOR} ${NODE_DELTA_ROBOT})

# Link target to our libraries
target_link_libraries(PickAndPlaceNode vision dataTypes)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()