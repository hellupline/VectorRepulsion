CXXFLAGS=-std=c++0x -Wall -O2 `pkg-config --cflags opencv`
LDLIBS=`pkg-config --libs opencv`
CC=g++
STRIP=strip

SOURCES=quadTree.cc dirVector.cc mapLearning.cc mapRender.cc
TARGET=VectorRepulsion
OBJS=$(SOURCES:.cc=.o)

all: $(TARGET)
$(TARGET): $(OBJS)
clean:
	rm -f $(OBJS)

