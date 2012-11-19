include(FindPackageHandleStandardArgs)

FIND_PROGRAM(SCONS_EXECUTABLE
  NAMES "scons"
  PATH /usr/bin
  DOC "SCons - Build system"
)

find_package_handle_standard_args(SCONS DEFAULT_MSG SCONS_EXECUTABLE)