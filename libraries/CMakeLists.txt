include_directories("Datatypes/include")
include_directories("Mast/include")

 add_subdirectories("dataTypes;motor;modbusController;utilities;deltaRobot;vision;mongo-cxx-driver;blackboardCppClient;libjson")
#add_subdirectories("virtualDeltaRobot;virtualModbusController;dataTypes;motor;utilities;vision;mongo-cxx-driver;blackboardCppClient;libjson")

# Adding all java libraries to building classpath
rexos_add_classpath(${CMAKE_CURRENT_SOURCE_DIR})