CXX      = g++
DEBUG    = -g
CXXFLAGS = -Wall $(DEBUG) -std=c++11
LDFLAGS  = -pthread
INCLUDES = -I${HOME}/Repos/linux-can-utils -I${HOME}/Miscellaneous/ros_lib

all: main cantest roscan rnet

cantest.o : cantest.c 
	g++ -c cantest.c -I${HOME}/Repos/linux-can-utils

#lib.o : lib.c 
#	gcc -c lib.c

cantest : lib.o cantest.o
	gcc -o cantest lib.o cantest.o -I${HOME}/Repos/linux-can-utils

main: HelloROS.cpp
	gcc -o main HelloROS.cpp -I../../ros_lib

roscan.o : roscan.cpp lib.o
	g++ -c roscan.cpp -I../../ros_lib -I${HOME}/Repos/linux-can-utils

roscan: roscan_test.cpp roscan.o lib.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o roscan roscan_test.cpp roscan.o lib.o ${INCLUDES}

rnet: rnet_node.cpp roscan.o lib.o
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o rnet rnet_node.cpp roscan.o lib.o ${INCLUDES}
