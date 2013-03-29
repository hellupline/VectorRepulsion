CXXFLAGS=-std=c++0x -Wall -O2 -I/home/hellupline/.AppCenter/apps/Aria/include `pkg-config --cflags opencv`
LDLIBS=-lAria -lpthread -ldl -L/home/hellupline/.AppCenter/apps/Aria/lib `pkg-config --libs opencv`

#TARGET=VectorRepulsion
TARGET=VectorRepulsion
OBJS=quadTree.o dirVector.o mapLearning.o

all: $(TARGET)
$(TARGET): $(OBJS)

clean:
	rm -f $(OBJS) $(TARGET)

