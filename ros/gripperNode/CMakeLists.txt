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

# Only add the target to the generated makefile if all external library dependencies 
# are satisfied. 
if(MODBUS_FOUND) 
	# Initialize ROS package to be in our build system. 
	lcv_rospack_init("grippernode" true true)

	# Add executables as documented by ROS
	rosbuild_add_executable(grippernode src/main.cpp)

	# Add dependencies to our libraries and other ROS packages in the same directory as this one. 
	add_dependencies(grippernode gripper)

	# Include and link to libraries
	include_directories(${MODBUS_INCLUDE_DIRS})
	target_link_libraries(grippernode ${MODBUS_LIBRARIES})

	# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
	include_directories("${LIB_GRIPPER_INCLUDE}")

	# Link target to our libraries
	target_link_libraries(grippernode gripper)
	
	# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
	lcv_rospack_end()

# Oh, let's just warn we omitted an target. 
else(MODBUS_FOUND)
	lcv_cannot_build("grippernode" "ROS/grippernode")
endif(MODBUS_FOUND)

