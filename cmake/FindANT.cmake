include(FindPackageHandleStandardArgs)

FIND_PROGRAM(ANT_FOUND
  NAMES "ant"
  PATH /usr/bin
  DOC "Ant - build generator for Java"
)

set(ANT_EXECUTABLE "${ANT_FOUND}")
find_package_handle_standard_args(ANT DEFAULT_MSG ANT_FOUND)

LIST(APPEND DEFAULT_JAR_DIRS
  /usr/share/java/ 
  /usr/lib/java/ 
  /usr/share/java 
  /usr/share/java/jar 
  /opt/java/lib 
  /usr/local/java 
  /usr/local/java/jar 
  /usr/local/share/java 
  /usr/local/share/java/jar 
  /usr/local/lib/java
)



