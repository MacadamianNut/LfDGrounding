###############################################################
#
# Purpose: Makefile for "hokeypokey"
# Author.: Cory Hayes (though I just modified it from a robotis file)
#
###############################################################

TARGET = hokeypokey

CXX = g++
INCLUDE_DIRS = -I/darwin/Linux/include -I/darwin/Framework/include
CXXFLAGS +=	-O2 -DLINUX -g -Wall -fmessage-length=0 $(INCLUDE_DIRS)
LIBS += -lpthread -lrt

OBJS = hokeypokey.o


all: darwin.a $(TARGET)

darwin.a:
	make -C /darwin/Linux/build

$(TARGET): $(OBJS) /darwin/Linux/lib/darwin.a
	$(CXX) -o $(TARGET) $(OBJS) /darwin/Linux/lib/darwin.a $(LIBS)
	
clean:
	rm -f $(OBJS) $(TARGET)





