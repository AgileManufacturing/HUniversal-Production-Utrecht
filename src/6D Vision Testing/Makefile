MINGW_PATH 			= C:\MinGW\bin
OPENCV_INCL_PATH 	= C:\Users\kjeld\Downloads\opencv\build\include

SOURCES				= $(wildcard *.cpp)
OBJECTS				= $(SOURCES:.cpp=.o)
CCFLAGS				= -std=c++11 -MMD
LDFLAGS				= -Llib
LDLIBS				= -lopencv_core310 -lopencv_imgproc310 -lopencv_highgui310 -lopencv_imgcodecs310
INCLUDES			= -I$(OPENCV_INCL_PATH)

Vision6D: $(OBJECTS)
	$(MINGW_PATH)/g++ $(CCFLAGS) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)
	
%.o: %.cpp
	$(MINGW_PATH)/g++ $(CCFLAGS) $(INCLUDES) -c -o $@ $<
	
-include $(OBJECTS:.o=.d)