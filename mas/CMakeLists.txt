##############################################################
# CMakeLists.txt for a ROS package within the rexos
# build system.
#
# Created by Dick van der Steen <dickvandersteen@gmail.com>, 15-11-2012
#
# 15-11-2012 Initial version
#
#############################################################

####### Add all ROS packages here
set(AGENTS dummyAgent spamAgent)

# add classpaths for all the agents
rexos_add_agents("${AGENTS}")