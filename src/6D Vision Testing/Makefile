SOURCES				= $(wildcard src/*.cpp)
OBJECTS				= $(SOURCES:.cpp=.o)
CCFLAGS				= -std=c++11 -MMD
LDFLAGS				= -LOpenCV/lib
LDLIBS				= -lopencv_core310 -lopencv_imgproc310 -lopencv_highgui310 -lopencv_imgcodecs310
INCLUDES			= -IOpenCV/include -Iincl/

.PHONY: clean

6DTool: $(OBJECTS)
	g++ $(CCFLAGS) $(LDFLAGS) -o $@ $(OBJECTS) $(LDLIBS)

clean: 
	del 6DTool.exe
	del src\*.o
	del src\*.d
	
%.o: %.cpp
	g++ $(CCFLAGS) $(INCLUDES) -c -o $@ $<
	
-include $(OBJECTS:.o=.d)