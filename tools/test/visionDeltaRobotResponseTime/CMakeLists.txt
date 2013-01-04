##############################################################
# CMakeLists.txt for a ROS package within the REXOS build system.
#
# Created by Koen Braham, 2012-12-18
#
# 2012-10-18 Initial version
#
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system.
rexos_rospack_init("visionDeltaRobotResponseTime" true true)

# Add executables as documented by ROS
rosbuild_add_executable(CrateLocatorNode src/CrateLocatorNode.cpp)
add_executable(CrateLocatorNode src/Crate.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(CrateLocatorNode vision)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_UTILITIES_INCLUDE} ${LIB_VISION_INCLUDE})

# Link target to our libraries
target_link_libraries(CrateLocatorNode vision utilities)


# Add executables as documented by ROS
rosbuild_add_executable(DeltaRobotNode src/DeltaRobotNode.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(DeltaRobotNode deltaRobot)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_DATATYPES_INCLUDE} ${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE} ${LIB_DELTAROBOT_INCLUDE} ${LIB_MOTOR_INCLUDE} ${LIB_MODBUS_CONTROLLER_INCLUDE})

# Link target to our libraries
target_link_libraries(DeltaRobotNode deltaRobot modbusController motor utilities)


# Add executables as documented by ROS
rosbuild_add_executable(FollowNode src/FollowNode.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one.
add_dependencies(FollowNode deltaRobotNode crateLocatorNode)

# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE} ${LIB_VISION_INCLUDE} ${NODE_CRATE_LOCATOR} ${NODE_DELTA_ROBOT})

# Link target to our libraries
target_link_libraries(FollowNode vision dataTypes)


# MUST be called after rexos_rospack_init to clean up after ROSBUILD!
rexos_rospack_end()
