

CXXFLAGS := -std=c++14 -O2

srcfiles := $(shell  find . -name "*.cpp" ! -name "DetKDecompMain.cpp")
localfiles := $(shell  find . -maxdepth 1 -name "*.cpp" ! -name "DetKDecompMain.cpp")
objects  := $(patsubst %.cpp, %.o, $(localfiles))
allobjects := $(patsubst %.cpp, %.o, $(srcfiles))

all: detkdecomp

detkdecomp: $(objects) DetKDecompMain.o | bin
	    $(CXX) $(CXXFLAGS) -o detkdecomp DetKDecompMain.o $(objects)

depend: .depend

bin:
	mkdir -p $@

.depend: $(srcfiles)
	    rm -f ./.depend
		    $(CXX) $(CXXFLAGS) -MM $^>>./.depend;

clean:
	    rm -f $(allobjects)
	    rm -f bin/*

dist-clean: clean
	    rm -f *~ .depend

include .depend

