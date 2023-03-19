CC = gcc
CXX = g++

INC = -Wall -ggdb
LDFLAGS = -lm -lpthread

SOURCES = serial_if.cpp

all: uavmain

uavmain: uavmain.cpp $(SOURCES)
	$(CXX) $(INC) -o $@ $^ $(LDFLAGS)


.PHONY: clean
clean:
	rm -f *.o uavmain
