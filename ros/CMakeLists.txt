####### Add all ROS packages here
set(ROSPACKS rosMast deltaRobotNode equipletNode crateLocatorNode followNode gripperNode)

### Don't change anything below, the part below checks if ROS is installed and configured correctly. 
######
rexos_check_ros()
if(ROS_OK)		
	add_subdirectories("${ROSPACKS}") # Add all ROS packages
else(ROS_OK)
	message("ROS packages cannot be build")
endif(ROS_OK)
