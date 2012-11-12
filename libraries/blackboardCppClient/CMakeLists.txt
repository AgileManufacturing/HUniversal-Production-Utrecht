# blackboardCppClient #
link_directories(${CMAKE_SOURCE_DIR}/libraries/mongo-cxx-driver)
rexos_add_library(blackboardCppClient)
include_directories(${MONGO_CXX_DRIVER})
target_link_libraries(blackboardCppClient mongoclient boost_thread-mt boost_filesystem boost_program_options boost_system)