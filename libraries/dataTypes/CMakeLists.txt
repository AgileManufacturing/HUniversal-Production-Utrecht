# dataTypes # 
if(OpenCV_FOUND)
	rexos_add_library(dataTypes)
	include_directories(${Boost_INCLUDE_DIRS})
	target_link_libraries(dataTypes ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS})
endif(OpenCV_FOUND)






