MINGW_PATH 			= C:\MinGW\bin
OPENCV_INCL_PATH 	= C:\Users\kjeld\Downloads\opencv\build\include
OPENCV_LIB_PATH 	= C:\Users\kjeld\Downloads\opencv\output\lib

SOURCES				= $(wildcard *.cpp)
OBJECTS				= $(SOURCES:.cpp=.o)
CCFLAGS				= -std=c++11 -MMD
LDFLAGS				= -L$(OPENCV_LIB_PATH)
LDLIBS				= -lopencv_core310 -lopencv_imgproc310 -lopencv_highgui310 -lopencv_imgcodecs310
INCLUDES			= -I$(OPENCV_INCL_PATH)


.PHONY: Vision6D

Vision6D: $(OBJECTS)
	$(MINGW_PATH)/g++ $(CCFLAGS) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)
	
%.o: %.cpp
	$(MINGW_PATH)/g++ $(CCFLAGS) $(INCLUDES) -c -o $@ $<
	
-include $(OBJECTS:.o=.d)