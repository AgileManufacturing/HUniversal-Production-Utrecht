############################################################################
 # External Libraries
 # 
 # Created by Pascal Muller <pascalmuller@gmail.com>, 16-04-2012
 # 
 # 16-04-2012 File added
 # 
 # == Description == 
 # This file attempts to find all external libraries, so this only has to happen once. 
 # 
##############################################################################


# == Find Doxygen == #
#      Sets DOXYGEN_FOUND and DOXYGEN_EXECUTABLE
find_package(Doxygen)

# == Find Java  == #
#      Sets Java_JAVAC_EXECUTABLE, Java_JAVA_EXECUTABLE, Java_JAR_EXECUTABLE and Java_VERSION_MINOR
find_package(Java)

# == Find ANT  == #
#      Sets ANT_EXECUTABLE and ANT_FOUND
#	Maybe we should skip this if java is not available
find_package(ANT)

# == Find Protobuf  == #
#      Sets PROTOBUF_INCLUDE_DIRS and PROTOBUF_LIBRARIES and PROTOBUF_FOUND
#	Maybe we should add protoc compiling to the build system. 
find_package(Protobuf)

# == Find Modbus  == #
#      Sets MODBUS_INCLUDE_DIRS, MODBUS_LIBRARIES
find_package(Modbus)

# == Find Boost  == #
# This should be installed if ROS is installed
find_package(Boost COMPONENTS system filesystem thread)

# == Find ZBar  == #
# Sets ZBAR_FOUND, ZBAR_INCLUDE_DIR, ZBAR_LIBRARIES
find_package(ZBar0)

# == Find Unicap  == #
# Sets UNICAP_FOUND, UNICAP_INCLUDE_DIRS, UNICAP_LIBRARIES
find_package(Unicap)

# == Find OpenCV  == #
#sets OpenCV_FOUND, OpenCV_INCLUDE_DIRS, OpenCV_LIBS

# OpenCV should automatically work because of ROS, but on some Ubuntu configurations
# it seems necessary to set OpenCV_DIR correctly. The next statement will try to 
# automatically guess this seting for ROS fuerte. 
if(NOT OpenCV_DIR)
	find_path(OpenCV_DIR "OpenCVConfig.cmake" DOC "Root directory of OpenCV" HINTS "/opt/ros/fuerte/share/OpenCV/")
endif(NOT OpenCV_DIR)
# Finally, try finding the package
find_package(OpenCV)

# == Find wxWidgets  == #
# Sets wxWidgets_FOUND, wxWidgets_LIBRARIES
find_package(wxWidgets COMPONENTS core base)
include("${wxWidgets_USE_FILE}")

# == Find GLUT  == #
# Sets GLUT_FOUND, GLUT_INCLUDE_DIR, GLUT_LIBRARIES
find_package(GLUT)

# == Find OpenGL  == #
# Sets OPENGL_FOUND, OPENGL_XMESA_FOUND, OPENGL_GLU_FOUND, OPENGL_INCLUDE_DIR, OPENGL_LIBRARIES
find_package(OpenGL)
