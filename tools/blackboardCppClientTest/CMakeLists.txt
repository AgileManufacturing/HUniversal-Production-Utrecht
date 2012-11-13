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
rexos_rospack_init("blackboardCppClientTest" true true)

# Add executables as documented by ROS
link_directories(${CMAKE_SOURCE_DIR}/libraries/mongo-cxx-driver/src)
rosbuild_add_executable(BlackboardCppClientTest src/BlackboardCppClientTest.cpp)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake

include_directories(${Boost_INCLUDE_DIRS} ${BLACKBOARD_CPP_CLIENT})

target_link_libraries(BlackboardCppClientTest mongoclient boost_thread-mt boost_filesystem boost_program_options boost_system ${Boost_SYSTEM_LIBRARY} blackboardCppClient)

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()
