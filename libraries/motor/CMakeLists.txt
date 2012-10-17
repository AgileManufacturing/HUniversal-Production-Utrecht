# Motor # 
if(${MODBUS_FOUND})
	rexos_add_library(motor)
	add_dependencies(motor modbusController utilities)
	include_directories(${MODBUS_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${LIB_MODBUS_CONTROLLER_INCLUDE} ${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE})
	target_link_libraries(motor ${MODBUS_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} modbusController utilities)	
else(${MODBUS_FOUND})
	rexos_cannot_build(motor, "libraries/motor")
endif()