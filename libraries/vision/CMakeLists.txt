# vision # 
if(OpenCV_FOUND AND ZBAR_FOUND)
	rexos_add_library(vision)
	include_directories(${ZBAR_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${LIB_DATATYPES_INCLUDE} ${LIB_UTILITIES_INCLUDE})
	target_link_libraries(vision ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${OpenCV_LIBS} ${ZBAR_LIBRARIES} utilities)
endif(OpenCV_FOUND AND ZBAR_FOUND)