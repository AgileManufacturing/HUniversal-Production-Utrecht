# DeltaRobot # 
if(${MODBUS_FOUND})
	rexos_add_library(deltaRobot)
	add_dependencies(modbusController utilities dataTypes)
	include_directories(${MODBUS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LIB_UTILITIES} ${LIB_DATATYPES_INCLUDE})
	target_link_libraries(deltaRobot ${MODBUS_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} utilities dataTypes)	
else(${MODBUS_FOUND})
	rexos_cannot_build(deltaRobot, "libraries/deltaRobot")
endif()
rexos_add_doc(deltaRobot)






