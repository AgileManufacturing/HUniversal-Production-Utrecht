## Add all ROS packages here ##
set(TOOLSPACKS keyboardControlNode)

# Add documentation targets
foreach(SINGLE_PACKDIR "${TOOLSPACKS}")
	add_lcv_doc(${SINGLE_PACKDIR})
endforeach(SINGLE_PACKDIR "${TOOLSPACKS}")

### Don't change anything below, the part below checks if ROS is installed and configured correctly. 
######
lcv_check_ros()
if(ROS_OK)		
	add_subdirectories("${TOOLSPACKS}") # Add all ROS packages
else(ROS_OK)
	message("ROS packages cannot be build")
endif(ROS_OK)
