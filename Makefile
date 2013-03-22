CFLAGS=-Wall -O2 `pkg-config --cflags opencv`
LDLIBS=`pkg-config --libs opencv`

all: render
