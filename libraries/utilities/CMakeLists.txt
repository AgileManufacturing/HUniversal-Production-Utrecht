## Configure packages ##

# xml_io # 
add_lcv_library(xml_io)
target_link_libraries(xml_io tinyxml)
add_lcv_doc(xml_io)

# AutoKeyStore #
add_lcv_library(AutoKeyStore)
add_lcv_doc(AutoKeyStore)
