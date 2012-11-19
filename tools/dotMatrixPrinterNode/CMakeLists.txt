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
rexos_rospack_init("dotMatrixPrinterNode" false false)

# Add executables as documented by ROS
rosbuild_add_executable(DotMatrixPrinterNode src/DotMatrixPrinterNode.cpp)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake

include_directories(${LIB_UTILITIES_INCLUDE} ${Boost_INCLUDE_DIRS} ${NODE_DELTA_ROBOT} ${NODE_IMAGE_TRANSFORMATION})

target_link_libraries(DotMatrixPrinterNode utilities ${Boost_SYSTEM_LIBRARY})

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()
