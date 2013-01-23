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


crexos_add_executable(BlackboardCppClientTest true "rexos_utilities;rexos_blackboard_cpp_client;roscpp;" "Boost")

