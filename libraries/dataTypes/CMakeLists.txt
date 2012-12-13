# dataTypes # 
if(OpenCV_FOUND)
	rexos_add_library(dataTypes)
	include_directories(${Boost_INCLUDE_DIRS} ${LIB_UTILITIES_INCLUDE})
	target_link_libraries(dataTypes ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS} utilities)
endif(OpenCV_FOUND)






