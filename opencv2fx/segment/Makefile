PLUGINOBJECTS = opencv2fx.o segment.o
PLUGINNAME = segment
PATHTOROOT = ../../openfx/Examples
VPATH = ..

include $(PATHTOROOT)/Makefile.master
CXXFLAGS += `pkg-config opencv --cflags`
LINKFLAGS += `pkg-config opencv --libs`
