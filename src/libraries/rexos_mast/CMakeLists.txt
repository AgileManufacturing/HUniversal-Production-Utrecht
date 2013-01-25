cmake_minimum_required(VERSION 2.8.3)
project(rexos_mast)

crexos_generate_messages_and_services("")
crexos_add_library(rexos_mast false "roscpp" "")
crexos_catkin_package()