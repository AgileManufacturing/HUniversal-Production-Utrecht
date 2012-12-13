# Unfortunately, we need this so the ROS cmake files know where to look for our libraries :(
# For every library we set global variables so our subjeprojects can find the includes and libraries they need.

# TODO: Find a nicer way to take care of this.

# add rexos libraries here 
# template: set(libraryname "${REXOS_SOURCE_DIR}library directory")
set(LIB_DATATYPES_INCLUDE "${REXOS_SOURCE_DIR}/libraries/dataTypes/include")
set(LIB_MAST_INCLUDE "${REXOS_SOURCE_DIR}/libraries/mast/include")
set(LIB_DELTAROBOT_INCLUDE "${REXOS_SOURCE_DIR}/libraries/deltaRobot/include")
set(LIB_MODBUS_CONTROLLER_INCLUDE "${REXOS_SOURCE_DIR}/libraries/modbusController/include")
set(LIB_UTILITIES_INCLUDE "${REXOS_SOURCE_DIR}/libraries/utilities/include")
set(LIB_MOTOR_INCLUDE "${REXOS_SOURCE_DIR}/libraries/motor/include")
set(LIB_VISION_INCLUDE "${REXOS_SOURCE_DIR}/libraries/vision/include")
set(LIB_INPUT_OUTPUT_INCLUDE  "${REXOS_SOURCE_DIR}/libraries/inputOutput/include")
set(NODE_DELTA_ROBOT "${REXOS_SOURCE_DIR}/ros/deltaRobotNode/include")
set(NODE_CRATE_LOCATOR "${REXOS_SOURCE_DIR}/ros/crateLocatorNode/include")
set(NODE_FOLLOW "${REXOS_SOURCE_DIR}/ros/followNode/include")
set(NODE_DOT_MATRIX_PRINTER "${REXOS_SOURCE_DIR}/tools/dotMatrixPrinterNode/include")
set(NODE_IMAGE_TRANSFORMATION "${REXOS_SOURCE_DIR}/tools/imageTransformationNode/include")
set(NODE_GRIPPER "${REXOS_SOURCE_DIR}/ros/gripperNode/include")
set(NODE_PICKANDPLACE "${REXOS_SOURCE_DIR}/tools/demos/pickAndPlaceNode/include")
