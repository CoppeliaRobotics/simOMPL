BOOST_DIR ?= $(shell find /usr/local/Cellar/boost -mindepth 1 -maxdepth 1 -type d | sort | tail -n1)
BOOST_CFLAGS = -I$(BOOST_DIR)/include
BOOST_LDLIBS = -L$(BOOST_DIR)/lib -lboost_system

OMPL_DIR ?= $(shell find /usr/local/Cellar/ompl -mindepth 1 -maxdepth 1 -type d | sort | tail -n1)
OMPL_CFLAGS = -I$(OMPL_DIR)/include
OMPL_LDLIBS = -L$(OMPL_DIR)/lib -lompl

# to override these variables, call make OMPL_DIR="/path/to/ompl" OMPL_LDLIBS="..."

# for when $PWD is a symlink:
PARENT_DIR = $(shell sh -c 'cd $$PWD/..; pwd')

CXXFLAGS = -ggdb -O0 -I$(PARENT_DIR)/include -Wall -Wno-unused -Wno-overloaded-virtual -Wno-sign-compare -fPIC $(BOOST_CFLAGS) $(OMPL_CFLAGS)
LDLIBS = -ggdb -O0 -lpthread -ldl $(BOOST_LDLIBS) $(OMPL_LDLIBS)

.PHONY: clean all install doc

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	EXT = so
	INSTALL_DIR ?= $(PARENT_DIR)/..
else
	CFLAGS += -D__APPLE__
	EXT = dylib
	INSTALL_DIR ?= $(PARENT_DIR)/../vrep.app/Contents/MacOS/
endif

all: libv_repExtOMPL.$(EXT) doc

doc: reference.html

reference.html: callbacks.xml callbacks.xsl
	xsltproc --path "$(PWD)" -o $@ $^

v_repExtOMPL.o: stubs.h

stubs.o: stubs.h stubs.cpp

stubs.h: callbacks.xml
	python external/v_repStubsGen/main.py -H $@ $<

stubs.cpp: callbacks.xml
	python external/v_repStubsGen/main.py -C $@ $<

libv_repExtOMPL.$(EXT): v_repExtOMPL.o stubs.o $(PARENT_DIR)/common/v_repLib.o
	$(CXX) $^ $(LDLIBS) -shared -o $@

clean:
	rm -f libv_repExtOMPL.$(EXT)
	rm -f *.o
	rm -f stubs.cpp stubs.h
	rm -f reference.html

install: all
	cp libv_repExtOMPL.$(EXT) $(INSTALL_DIR)
