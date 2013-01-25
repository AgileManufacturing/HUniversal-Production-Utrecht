cmake_minimum_required(VERSION 2.8.3)
project(delta_robot_node)

crexos_add_library(delta_robot_node true "roscpp;rexos_mast;rexos_std_msgs;rexos_std_srvs;rexos_datatypes;rexos_delta_robot;Libjson" "")
crexos_catkin_package()