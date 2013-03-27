# - Try to find Libunicap
# Once done this will define
#  UNICAP_FOUND - System has Libunicap
#  UNICAP_INCLUDE_DIRS - The Libunicap include directories
#  UNICAP_LIBRARIES - The libraries needed to use Libunicap
#  UNICAP_DEFINITIONS - Compiler switches required for using Libunicap

find_package(PkgConfig)
pkg_check_modules(PC_UNICAP QUIET libunicap)
set(UNICAP_DEFINITIONS ${PC_LIBUNICAP_CFLAGS_OTHER})

find_path(UNICAP_INCLUDE_DIRS unicap.h
          HINTS ${PC_LIBUNICAP_INCLUDEDIR} ${PC_LIBUNICAP_INCLUDE_DIRS}
          PATH_SUFFIXES unicap )

find_library(UNICAP_LIBRARIES NAMES unicap libunicap
             HINTS ${PC_LIBUNICAP_LIBDIR} ${PC_LIBUNICAP_LIBRARY_DIRS} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set UNICAP_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(UNICAP "Could not find libunicap" UNICAP_LIBRARIES UNICAP_INCLUDE_DIRS)


mark_as_advanced(UNICAP_INCLUDE_DIRS UNICAP_LIBRARIES)
