# - Try to find Jsoncpp
# Once done, this will define
#
#  Jsoncpp_FOUND - system has Jsoncpp
#  Jsoncpp_INCLUDE_DIRS - the Jsoncpp include directories
#  Jsoncpp_LIBRARIES - link these to use Jsoncpp

#include(LibFindMacros)

# Use pkg-config to get hints about paths
#libfind_pkg_check_modules(Jsoncpp_PKGCONF jsoncpp)

# Include dir
find_path(Jsoncpp_INCLUDE_DIR
  NAMES json/features.h
  PATH_SUFFIXES jsoncpp
  PATHS ${Jsoncpp_PKGCONF_INCLUDE_DIRS} # /usr/include/jsoncpp/json
)

# Finally the library itself
find_library(Jsoncpp_LIBRARIES
  NAMES jsoncpp
  PATHS ${Jsoncpp_PKGCONF_LIBRARY_DIRS}
#  PATH ./jsoncpp/
)
if(Jsoncpp_INCLUDE_DIR AND Jsoncpp_LIBRARIES)
  message(STATUS "Found JsonCPP: ${Jsoncpp_INCLUDE_DIR}, ${Jsoncpp_LIBRARIES}")
else(Jsoncpp_INCLUDE_DIR AND Jsoncpp_LIBRARIES)
  message(STATUS "jsonCPP not found.")
endif(Jsoncpp_INCLUDE_DIR AND Jsoncpp_LIBRARIES)

set(Jsoncpp_PROCESS_INCLUDES Jsoncpp_INCLUDE_DIR)
set(Jsoncpp_PROCESS_LIBS Jsoncpp_LIBRARY)
