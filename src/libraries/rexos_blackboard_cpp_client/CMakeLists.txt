# blackboardCppClient #
cmake_minimum_required(VERSION 2.8.3)
project(rexos_blackboard_cpp_client)

crexos_add_library(rexos_blackboard_cpp_client false "rexos_utilities" "Mongo;Boost")

crexos_catkin_package()