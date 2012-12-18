include_directories("Datatypes/include")
include_directories("Mast/include")

add_subdirectories("blackboardCppClient;dataTypes;deltaRobot;libjson;modbusController;motor;utilities;vision;mongo-cxx-driver;")

# Adding all java libraries to building classpath
rexos_add_classpath(${CMAKE_CURRENT_SOURCE_DIR})