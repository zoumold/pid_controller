# Compiler and flags
CXX = g++
CXXFLAGS = -Wall

# Targets
TARGET = example
OBJS = example.o controller.o

# Build the final executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Compile example.cpp
example.o: example.cpp controller.hpp
	$(CXX) $(CXXFLAGS) -c example.cpp

# Compile controller.cpp
controller.o: controller.cpp controller.hpp
	$(CXX) $(CXXFLAGS) -c controller.cpp

# Clean build files
.PHONY: clean
clean:
	rm -f $(TARGET) *.o
