CC=g++
CXXFLAGS=-std=c++14 -Wall -Wextra -pedantic
LDLIBS=-lprotobuf

CPPOBJS=sensor.pb.o sensor_reading.o
OBJS=$(addprefix cpp/, $(CPPOBJS))
BIN=sensor_reading

REQUIREMENTS=requirements.txt
TRASH=**/*.pyc **/__pycache__
PROTOBUF=python/test_pb2.py cpp/test.pb.h cpp/test.pb.cc

all: pb-python pb-cpp $(BIN)

pb-python:
	protoc -I=. --python_out=./python ./test.proto

pb-cpp:
	protoc -I=. --cpp_out=./cpp ./test.proto

init:
	pip3 install -r $(REQUIREMENTS)

build: pb-cpp $(BIN)

$(BIN): $(OBJS)
	$(CC) $(CXXFLAGS) -o $@ $^ $(LDLIBS)

debug: CXXFLAGS += -O0 -g
debug: $(BIN)

clean:
	$(RM) -r $(TRASH)
	$(RM) $(OBJS)
	$(RM) $(BIN)
	$(RM) $(PROTOBUF)

.PHONY: init clean all pb-python pb-cpp debug build