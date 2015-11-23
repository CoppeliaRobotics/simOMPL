BOOST_DIR ?= /usr/local/Cellar/boost/1.59.0
BOOST_CFLAGS = -I$(BOOST_DIR)/include
BOOST_LDLIBS = -L$(BOOST_DIR)/lib -lboost_system

OMPL_DIR ?= $(HOME)/ompl
OMPL_CFLAGS = -I$(OMPL_DIR)/src
OMPL_LDLIBS = -L$(OMPL_DIR)/build/Debug/lib -lompl

CXXFLAGS = -ggdb -O0 -I../include -Wall -Wno-unused -Wno-overloaded-virtual -Wno-sign-compare -fPIC -static $(BOOST_CFLAGS) $(OMPL_CFLAGS)
LDLIBS = -ggdb -O0 -lpthread -ldl -shared $(BOOST_LDLIBS) $(OMPL_LDLIBS)

.PHONY: clean all install

OS = $(shell uname -s)
ifeq ($(OS), Linux)
	CFLAGS += -D__linux
	EXT = so
	INSTALL_DIR ?= ../..
else
	CFLAGS += -D__APPLE__
	EXT = dylib
	INSTALL_DIR ?= ../../vrep.app/Contents/MacOS/
endif

all: libv_repExtOMPL.$(EXT)

libv_repExtOMPL.$(EXT): v_repExtOMPL.o v_repLib.o
	$(CXX) $^ $(LDLIBS) -o $@

clean:
	rm -f libv_repExtOMPL.$(EXT) *.o

install: all
	cp libv_repExtOMPL.$(EXT) $(INSTALL_DIR)
