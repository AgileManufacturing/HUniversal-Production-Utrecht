SOURCES				= $(wildcard *.cpp)
OBJECTS				= $(SOURCES:.cpp=.o)
CCFLAGS				= -std=c++11 -MMD
LDFLAGS				= -Llib
LDLIBS				= -lopencv_core310 -lopencv_imgproc310 -lopencv_highgui310 -lopencv_imgcodecs310
INCLUDES			= -Iinclude

6DTool: $(OBJECTS)
	g++ $(CCFLAGS) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)
	
%.o: %.cpp
	g++ $(CCFLAGS) $(INCLUDES) -c -o $@ $<
	
-include $(OBJECTS:.o=.d)