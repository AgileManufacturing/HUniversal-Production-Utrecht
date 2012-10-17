# Unfortunately, we need this so the ROS cmake files know where to look for our libraries :(
# For every library we set global variables so our subjeprojects can find the includes and libraries they need.

# TODO: Find a nicer way to take care of this.



#set(LIB_GRIPPER_INCLUDE "${LCV_SOURCE_DIR}/Libraries/Devices/gripper/include")
#set(LIB_PCRCTRANSFORMATION_INCLUDE "${LCV_SOURCE_DIR}/Libraries/Vision/PixelCord_ReallifeCord_Transformation/include")
#set(LIB_CAMERACALIBRATION_INCLUDE "${LCV_SOURCE_DIR}/Libraries/Vision/CameraCalibration/include")
#set(LIB_UNICAPCVBRIDGE_INCLUDE "${LCV_SOURCE_DIR}/Libraries/Vision/unicap_cv_bridge/include")
#set(LIB_FIDUCIAL_INCLUDE "${LCV_SOURCE_DIR}/Libraries/Vision/Fiducial/include")
#set(LIB_XMLIO_INCLUDE "${LCV_SOURCE_DIR}/Libraries/Utilities/xml_io/include")
#set(LIB_AUTOKEYSTORE_INCLUDE "${LCV_SOURCE_DIR}/Libraries/Utilities/AutoKeyStore/include")
#set(LIB_BLACKBOARDCLIENT_INCLUDE "${LCV_SOURCE_DIR}/Libraries/BlackboardClient/include")


# add rexos libraries here 
# template: set(libraryname "${REXOS_SOURCE_DIR}library directory")
set(LIB_DATATYPES_INCLUDE "${REXOS_SOURCE_DIR}/libraries/dataTypes/include")
set(LIB_DELTAROBOT_INCLUDE "${REXOS_SOURCE_DIR}/libraries/deltaRobot/include")
set(LIB_MODBUS_CONTROLLER_INCLUDE "${REXOS_SOURCE_DIR}/libraries/modbusController/include")
set(LIB_UTILITIES_INCLUDE "${REXOS_SOURCE_DIR}/libraries/utilities/include")
set(LIB_MOTOR_INCLUDE "${REXOS_SOURCE_DIR}/libraries/motor/include")
set(NODE_DELTA_ROBOT "${REXOS_SOURCE_DIR}/ros/deltaRobotNode/include")
