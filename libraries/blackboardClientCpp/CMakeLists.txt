# BlackBoardClient # 

rexos_add_library(blackboardClientCpp)

target_link_libraries(blackboardClientCpp autoKeyStoreCppClient)

include_directories(${LIB_AUTOKEYSTORE_INCLUDE})
