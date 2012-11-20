# ModbusController # 
if(${MODBUS_FOUND})
	rexos_add_library(modbusController)
	add_dependencies(modbusController utilities)
	include_directories(${MODBUS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LIB_UTILITIES_INCLUDE})
	target_link_libraries(modbusController ${MODBUS_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} utilities)	
else(${MODBUS_FOUND})
	rexos_cannot_build(modbusController, "libraries/modbusController")
endif()