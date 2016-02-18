BOOST_DIR ?= /usr/local/Cellar/boost/1.59.0
BOOST_CFLAGS = -I$(BOOST_DIR)/include
BOOST_LDLIBS = -L$(BOOST_DIR)/lib -lboost_system

OMPL_DIR ?= /usr/local/Cellar/ompl/1.1.0
OMPL_CFLAGS = -I$(OMPL_DIR)/include
OMPL_LDLIBS = -L$(OMPL_DIR)/lib -lompl

# to override these variables, call make OMPL_DIR="/path/to/ompl" OMPL_LDLIBS="..."

CXXFLAGS = -ggdb -O0 -I../include -Wall -Wno-unused -Wno-overloaded-virtual -Wno-sign-compare -fPIC $(BOOST_CFLAGS) $(OMPL_CFLAGS)
LDLIBS = -ggdb -O0 -lpthread -ldl $(BOOST_LDLIBS) $(OMPL_LDLIBS)

.PHONY: clean all install doc

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

all: libv_repExtOMPL.$(EXT) doc

doc: reference.html

reference.html: callbacks.xml callbacks.xsl
	saxon -s:callbacks.xml -a:on -o:$@

v_repExtOMPL.o: stubs.h

stubs.o: stubs.h stubs.cpp

stubs.h: callbacks.xml generate_stubs.py
	python generate_stubs.py -h callbacks.xml > stubs.h.tmp
	mv stubs.h.tmp stubs.h

stubs.cpp: callbacks.xml generate_stubs.py
	python generate_stubs.py -c callbacks.xml > stubs.cpp.tmp
	mv stubs.cpp.tmp stubs.cpp

libv_repExtOMPL.$(EXT): v_repExtOMPL.o stubs.o ../common/v_repLib.o ../common/luaFunctionData.o ../common/luaFunctionDataItem.o
	$(CXX) $^ $(LDLIBS) -shared -o $@

clean:
	rm -f libv_repExtOMPL.$(EXT)
	rm -f *.o
	rm -f stubs.cpp stubs.cpp.tmp stubs.h stubs.h.tmp
	rm -f reference.html

install: all
	cp libv_repExtOMPL.$(EXT) $(INSTALL_DIR)
