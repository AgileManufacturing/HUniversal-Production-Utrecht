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
find_path(JSONCPP_INCLUDE_DIR
  NAMES json/features.h
  PATH_SUFFIXES jsoncpp
  PATHS ${Jsoncpp_PKGCONF_INCLUDE_DIRS} # /usr/include/jsoncpp/json
)

# Finally the library itself
find_library(JSONCPP_LIBRARIES
  NAMES jsoncpp
  PATHS ${Jsoncpp_PKGCONF_LIBRARY_DIRS}
#  PATH ./jsoncpp/
)
if(JSONCPP_INCLUDE_DIR AND JSONCPP_LIBRARIES)
  message(STATUS "Found JSONCPP: ${JSONCPP_INCLUDE_DIR}, ${JSONCPP_LIBRARIES}")
else(JSONCPP_INCLUDE_DIR AND JSONCPP_LIBRARIES)
  message(STATUS "JSONCPP not found.")
endif(JSONCPP_INCLUDE_DIR AND JSONCPP_LIBRARIES)

set(JSONCPP_PROCESS_INCLUDES JSONCPP_INCLUDE_DIR)
set(JSONCPP_PROCESS_LIBS JSONCPP_LIBRARIES)
