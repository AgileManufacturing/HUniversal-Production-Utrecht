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
rexos_rospack_init("gripperTestNode" false true)

# Add executables as documented by ROS
rosbuild_add_executable(GripperTestNode src/GripperTestNode.cpp src/Gripper.cpp)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake

include_directories(${LIB_UTILITIES_INCLUDE} ${Boost_INCLUDE_DIRS})

target_link_libraries(GripperTestNode utilities ${Boost_SYSTEM_LIBRARY})

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()
