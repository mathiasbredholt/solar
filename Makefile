CXX = g++
CXXFLAGS = -g -Wall -std=c++14 -pthread

all: solar.o tinyosc.o
	$(CXX) $(CXXFLAGS) -o build/solar build/solar.o build/tinyosc.o

tinyosc.o: tinyosc/tinyosc.c
	$(CXX) $(CFLAGS) -o build/tinyosc.o -c tinyosc/tinyosc.c

solar.o: solar.cpp
	$(CXX) $(CXXFLAGS) -o build/solar.o -c solar.cpp