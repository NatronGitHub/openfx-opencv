PLUGINOBJECTS = \
VectorGenerator.o \
GenericOpenCVPlugin.o \
ofxsLut.o

PLUGINNAME = Misc

# ls */*png|sed -e 's!^!../!' -e 's!$! \\!'
RESOURCES = \
#../VectorGenerator/net.sf.openfx.VectorGenerator.png \

VPATH += \
../VectorGenerator

include ../Makefile.master

CXXFLAGS += \
-I. \
-I../VectorGenerator

