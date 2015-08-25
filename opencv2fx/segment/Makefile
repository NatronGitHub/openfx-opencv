PLUGINOBJECTS = opencv2fx.o segment.o
PLUGINNAME = segment
PATHTOROOT = ../../openfx/Examples
VPATH = ..

include $(PATHTOROOT)/Makefile.master
CXXFLAGS += $(shell pkg-config opencv --cflags) -I..
LINKFLAGS += $(shell pkg-config opencv --libs)
