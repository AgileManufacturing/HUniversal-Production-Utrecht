##############################################################################
 # CMakeLists for the LCV project. 
 # 
 # Created by Pascal Muller <pascalmuller@gmail.com>, 07-02-2012
 # 
 # 09-02-2012 Pascal Muller		Initial CMake support
 # 16-04-2012 Pascal Muller 	Changes to allow for Library loading in LibrariesExport.cmake
 # 19-09-2012 KBraham			Minified for fresh autumn start.
 # 
 # == Description == 
 # This file is used for generating a makefile for our project. This is done 
 # by a tool called
 # 
 # IMPORTANT! Look at this cmake/macros.cmake for important documentation!
##############################################################################

######## Configuration ##########

set(REXOS_PARTS libraries ros tools) # TODO: Agents Libraries ROS

# Note: Set buildtype with cmake -D CMAKE_BUILD_TYPE=<Debug|Release> <source>
# It might be a smart idea to make two out-of-source build directories, 
# one for the debug build, 

###### End configuration ########


# I'm not actually sure if we really need CMake 2.8, but this was the version
# Ubuntu installed, so just to be sure it'll work. 
cmake_minimum_required (VERSION 2.8)

# Set the name of our project to LCV and require a C and C++ compiler. 
project (REXOS CXX C) # TODO, add java??

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# Include our own macros
include(cmake/macros.cmake) 

# Check for all external libraries and export our own library includes
include(cmake/ExternalLibraries.cmake)
include(libraries/LibrariesExport.cmake) 

# I like to see all the warnings, no matter what happens. 
# This is actually a bit hacky and probably it should be done differently
if(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
endif()

# Recursively add CMakeLists.txt rules for all REXOS_PARTS (directories)
add_subdirectories("${REXOS_PARTS}")
rexos_add_doc(REXOS)

