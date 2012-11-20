##############################################################
# CMakeLists.txt for a ROS package within the rexos
# build system.
#
# Created by Dick van der Steen <dickvandersteen@gmail.com>, 15-11-2012
#
# 15-11-2012 Initial version
#
#############################################################

####### Add all ROS packages here
set(ROSPACKS rosMast rexosStdMsgs rexosStdSrvs deltaRobotNode equipletNode crateLocatorNode followNode )

### Don't change anything below, the part below checks if ROS is installed and configured correctly. 
rexos_check_ros()
if(ROS_OK)		
	add_subdirectories("${ROSPACKS}") # Add all ROS packages
else(ROS_OK)
	message("ROS packages cannot be build")
endif(ROS_OK)
