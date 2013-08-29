CXXFLAGS = -I../include -I"D:\c\OpenCV2.1\include\opencv" -march=i686 -O3
LDFLAGS = -LD:\c\OpenCV2.1\lib -lcxcore210 -lcv210 -lcvaux210

all: inpaint.ofx segment.ofx

opencv2fx.o : opencv2fx.cpp
	$(CXX) $(CXXFLAGS) -c opencv2fx.cpp

inpaint.o : inpaint.cpp
	$(CXX) $(CXXFLAGS) -c inpaint.cpp

inpaint.ofx : inpaint.o opencv2fx.o
	$(CXX) -shared inpaint.o opencv2fx.o -o inpaint.ofx $(LDFLAGS)
	strip -s inpaint.ofx

segment.ofx : segment.o opencv2fx.o
	$(CXX) -shared segment.o opencv2fx.o -o segment.ofx $(LDFLAGS)
	strip -s segment.ofx

clean:
	del *.o
	del *.*~
	del *~.*

install: segment.ofx inpaint.ofx
	copy *.ofx "c:\Program Files\Sony\Vegas Pro 10.0\OFX Video Plug-Ins\Filters.ofx.bundle\Contents\Win32"
