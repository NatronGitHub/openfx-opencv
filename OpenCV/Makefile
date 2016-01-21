PLUGINOBJECTS = \
VectorGenerator.o \
GenericOpenCVPlugin.o \
ofxsLut.o

PLUGINNAME = OpenCV

# ls */*png|sed -e 's!^!../!' -e 's!$! \\!'
RESOURCES = \
#../VectorGenerator/net.sf.openfx.VectorGenerator.png \

TOP_SRCDIR = ..

VPATH += \
$(TOP_SRCDIR)/VectorGenerator

include $(TOP_SRCDIR)/Makefile.master

CXXFLAGS += \
-I. \
-I$(TOP_SRCDIR)/VectorGenerator

