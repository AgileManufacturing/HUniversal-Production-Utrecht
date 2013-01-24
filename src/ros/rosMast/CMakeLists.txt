cmake_minimum_required(VERSION 2.8.3)
project(rosMast)

crexos_generate_messages_and_services("")
crexos_add_library(rosMast false "roscpp" "")
crexos_catkin_package()