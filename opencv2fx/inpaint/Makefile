PLUGINOBJECTS = opencv2fx.o inpaint.o
PLUGINNAME = inpaint
PATHTOROOT = ../../openfx/Examples
VPATH = ..

include $(PATHTOROOT)/Makefile.master
CXXFLAGS += `pkg-config opencv --cflags` -I..
LINKFLAGS += `pkg-config opencv --libs`
