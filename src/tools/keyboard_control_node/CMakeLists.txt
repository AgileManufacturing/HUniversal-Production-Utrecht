cmake_minimum_required(VERSION 2.8.3)
project(keyboard_control_node)

crexos_add_executable(keyboard_control_node "src/KeyBoardControlNode.cpp" false "rexos_delta_robot_node" "")
