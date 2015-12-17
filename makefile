BOOST_DIR ?= /usr/local/Cellar/boost/1.59.0
BOOST_CFLAGS = -I$(BOOST_DIR)/include
BOOST_LDLIBS = -L$(BOOST_DIR)/lib -lboost_system

OMPL_DIR ?= $(HOME)/ompl
OMPL_CFLAGS = -I$(OMPL_DIR)/src
OMPL_LDLIBS = -L$(OMPL_DIR)/build/Debug/lib -lompl

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

generate_reference_xml: v_repExtOMPL.cpp v_repLib.cpp
	$(CXX) $(CXXFLAGS) -DGENERATE_DOC v_repExtOMPL.cpp $(LDLIBS) -o $@

reference.html: v_repExtOMPL.cpp format_docs.py generate_reference_xml
	./generate_reference_xml | ./format_docs.py > $@.tmp
	mv $@.tmp $@

libv_repExtOMPL.$(EXT): v_repExtOMPL.o v_repLib.o
	$(CXX) $^ $(LDLIBS) -shared -o $@

clean:
	rm -f libv_repExtOMPL.$(EXT)
	rm -f *.o
	rm -f generate_reference_xml
	rm -f reference.html
	rm -f reference.html.tmp
	rm -rf generate_reference_xml.dSYM

install: all
	cp libv_repExtOMPL.$(EXT) $(INSTALL_DIR)
