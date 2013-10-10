# inputOutput # 
if(${MODBUS_FOUND})
	rexos_add_library(inputOutput)
	add_dependencies(inputOutput modbusController)
	include_directories(${MODBUS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LIB_MODBUS_CONTROLLER_INCLUDE} ${LIB_UTILITIES_INCLUDE})
	target_link_libraries(inputOutput ${MODBUS_LIBRARIES} ${Boost_SYSTEM_LIBRARY} modbusController utilities)	
else(${MODBUS_FOUND})
	rexos_cannot_build(inputOutput, "libraries/inputOutput")
endif()