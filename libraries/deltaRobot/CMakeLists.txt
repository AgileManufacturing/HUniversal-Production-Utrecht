# CameraCalibration # 
if(${MODBUS_FOUND})
	add_lcv_library(huniplacer)
	include_directories(${MODBUS_INCLUDE_DIRS})
	include_directories(${Boost_INCLUDE_DIRS})
	target_link_libraries(huniplacer ${MODBUS_LIBRARIES} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})	
else(${MODBUS_FOUND})
	lcv_cannot_build(huniplacer, "Libraries/Deltarobot/huniplacer")
endif()
add_lcv_doc(huniplacer)






