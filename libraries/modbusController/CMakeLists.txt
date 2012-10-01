# CameraCalibration # 
if(${MODBUS_FOUND})
	rexos_add_library(modbusController)
	include_directories(${MODBUS_INCLUDE_DIRS})
	include_directories(${Boost_INCLUDE_DIRS})
	target_link_libraries(modbusController ${MODBUS_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})	
else(${MODBUS_FOUND})
	rexos_cannot_build(modbusController, "libraries/modbusController")
endif()
rexos_add_doc(modbusController)






