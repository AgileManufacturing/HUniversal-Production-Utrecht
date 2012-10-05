# DeltaRobot # 
if(${MODBUS_FOUND})
	rexos_add_library(deltaRobot)
	add_dependencies(deltaRobot utilities motor modbusController)
	include_directories(${MODBUS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LIB_UTILITIES_INCLUDE} ${LIB_MODBUS_CONTROLLER_INCLUDE}  ${LIB_DATATYPES_INCLUDE} ${LIB_MOTOR_INCLUDE})
	target_link_libraries(deltaRobot ${MODBUS_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} modbusController utilities motor)	
else(${MODBUS_FOUND})
	rexos_cannot_build(deltaRobot, "libraries/deltaRobot")
endif()






