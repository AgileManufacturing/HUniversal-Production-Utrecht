cmake_minimum_required(VERSION 2.8.3)
project(crate_locator_node)

crexos_generate_messages_and_services("")
crexos_add_library(crate_locator_node false "roscpp;image_transport;opencv2;sensor_msgs;rexos_datatypes;rexos_vision" "")
crexos_catkin_package()