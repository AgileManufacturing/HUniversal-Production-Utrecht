##############################################################
# CMakeLists.txt for a ROS package within the rexos
# build system.
#
# Created by Dick van der Steen & Dennis Koole
# <dickvandersteen@gmail.com, dennis.koole@gmail.com>
#
#############################################################

## Add all ROS packages here ##
set(TOOLSPACKS gripperTestNode blackboardCppClientTest deltaRobotTest)

### Don't change anything below, the part below checks if ROS is installed and configured correctly. 
rexos_check_ros()
if(ROS_OK)		
	add_subdirectories("${TOOLSPACKS}") # Add all ROS packages
else(ROS_OK)
	message("ROS packages cannot be build")
endif(ROS_OK)


