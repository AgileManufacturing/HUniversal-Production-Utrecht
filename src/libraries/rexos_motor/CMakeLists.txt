cmake_minimum_required(VERSION 2.8.3)
project(rexos_motor)

crexos_add_library(rexos_motor false "rexos_modbus;rexos_datatypes" "Boost")
crexos_catkin_package()