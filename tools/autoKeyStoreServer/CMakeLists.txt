add_executable_jar(AutoKeyStoreServer nl.hu.autokeystore.server.AutoKeyStoreServer src/nl/hu/autokeystore/server/*.java)
create_javadoc(AutoKeyStoreServer
	PACKAGES nl.hu.autokeystore.server
	SOURCEPATH ${CMAKE_CURRENT_SOURCE_DIR}/src
	CLASSPATH ${CMAKE_JAVA_INCLUDE_PATH}
	WINDOWTITLE "AutoKeyStoreServer"
	DOCTITLE "<h1>AutoKeyStoreServer</h1>"
	AUTHOR TRUE
	USE TRUE
	VERSION TRUE
)
