## Configure packages ##

# xml_io # 
#rexos_add_library(xml_io)
#target_link_libraries(xml_io tinyxml)
#rexos_add_doc(xml_io)

# AutoKeyStore #
#rexos_add_library(AutoKeyStore)
#rexos_add_doc(AutoKeyStore)

# Utilities # 
rexos_add_library(utilities)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(utilities ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})
rexos_add_doc(utilities)
