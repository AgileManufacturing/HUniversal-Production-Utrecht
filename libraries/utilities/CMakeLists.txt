## Configure packages ##

# xml_io # 
#rexos_add_library(xml_io)
#target_link_libraries(xml_io tinyxml)

# Utilities # 
rexos_add_library(utilities)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(utilities rt ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
