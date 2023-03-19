INCLUDE_DIRS = 
LIB_DIRS = 
CC=g++

CDEFS=
CFLAGS= -O0 -pg -g $(INCLUDE_DIRS) $(CDEFS) -pthread
LIBS= -lrt
CPPLIBS= -L/usr/lib -lopencv_core -lopencv_flann -lopencv_video 

HFILES= 
CFILES= 
CPPFILES= capture.cpp

SRCS= ${HFILES} ${CFILES}
CPPOBJS= ${CPPFILES:.cpp=.o}

all:	rt_image 

clean:
	-rm -f *.o *.d
	-rm -f capture

distclean:
	-rm -f *.o *.d

rt_image: capture.o
	$(CC) $(LDFLAGS) $(CFLAGS) -o rt_image capture.o `pkg-config --libs opencv` $(CPPLIBS)

depend:

.c.o:
	$(CC) $(CFLAGS) -c $<

.cpp.o:
	$(CC) $(CFLAGS) -c $<
