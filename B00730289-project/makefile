# makefile to compile lab asignments.
# Inaki Rano i.rano@ulster.ac.uk
#

# Set compiler and compiling options
CXX = g++
CXXFLAGS = `pkg-config --cflags playerc++`
LDFLAGS = `pkg-config --libs playerc++`

# Source files and program to compile
TARGET = office-wander
SRC = office-wander.cc
OBJ = $(SRC:.cc=.o)

# Compilation rules
%: %.o
	$(CXX) $< -o $@ $(LDFLAGS)

office-wander: office-wander.o
office-wander: office-wander.cc

clean:
	rm $(OBJ) $(TARGET)