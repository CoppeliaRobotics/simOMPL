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

generate_reference_xml.cpp: v_repExtOMPL.cpp
	cp v_repExtOMPL.cpp $@

generate_reference_xml.o: CXXFLAGS+=-DGENERATE_DOC

generate_reference_xml: generate_reference_xml.o ../common/v_repLib.o ../common/luaFunctionData.o ../common/luaFunctionDataItem.o
	$(CXX) $^ $(LDLIBS) -o $@

reference.xml: generate_reference_xml v_repExtOMPL.cpp
	./generate_reference_xml > $@.tmp
	mv $@.tmp $@

reference.html: reference.xml reference.xsl
	saxon -s:reference.xml -a:on -o:$@

libv_repExtOMPL.$(EXT): v_repExtOMPL.o ../common/v_repLib.o ../common/luaFunctionData.o ../common/luaFunctionDataItem.o
	$(CXX) $^ $(LDLIBS) -shared -o $@

clean:
	rm -f libv_repExtOMPL.$(EXT)
	rm -f *.o
	rm -f generate_reference_xml.cpp generate_reference_xml
	rm -rf generate_reference_xml.dSYM
	rm -f reference.html
	rm -f reference.xml reference.xml.tmp

install: all
	cp libv_repExtOMPL.$(EXT) $(INSTALL_DIR)
