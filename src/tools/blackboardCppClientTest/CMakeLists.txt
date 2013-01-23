##############################################################
# CMakeLists.txt for a ROS package within the low-cost-vision
# build system.
#
# Created by Pascal Muller <pascalmuller@gmail.com>, 09-02-2012
#
# 09-02-2012 Initial version
#
#############################################################


cmake_minimum_required(VERSION 2.8.3)
project(BlackboardCppClientTest)
find_package(catkin REQUIRED roscpp rexos_blackboard_cpp_client rexos_utilities)
find_package(Boost REQUIRED COMPONENTS system)




crexos_add_executable(BlackboardCppClientTest src/BlackboardCppClientTest.cpp true "rexos_utilities;rexos_blackboard_cpp_client;roscpp" "Boost")

