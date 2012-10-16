# inputOutputController # 
if(${MODBUS_FOUND})
	rexos_add_library(inputOutputController)
	add_dependencies(inputOutputController modbusController)
	include_directories(${MODBUS_INCLUDE_DIRS} ${LIB_MODBUS_CONTROLLER_INCLUDE})
	target_link_libraries(inputOutputController ${MODBUS_LIBRARIES} modbusController)	
else(${MODBUS_FOUND})
	rexos_cannot_build(inputOutputController, "libraries/inputOutputController")
endif()