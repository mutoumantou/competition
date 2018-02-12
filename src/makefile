IDIR = ../include
CC = g++
CFLAGS=-I$(IDIR)

GTKFLAGS = `pkg-config --cflags gtk+-3.0`
GTKLIBS  = `pkg-config --libs gtk+-3.0` `pkg-config --cflags --libs opencv --libs` -rdynamic

ODIR = obj
LDIR = ../lib

LIBS = -lm -ldc1394 # -l826_64

_DEPS = GUI_master.hpp general_header.hpp vision.hpp FWcamera.hpp actuation.hpp keyboard.hpp s826.hpp s826api.h
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = hellomake.o GUI_master.o vision.o FWcamera.o general_fun.o actuation.o keyboard.o s826_dummy.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

$(ODIR)/%.o: %.cpp $(DEPS)
	$(CC) -c $(GTKFLAGS) -o $@ $< $(GTKLIBS) $(CFLAGS) $(LIBS)

hellomake: $(OBJ)
	$(CC) $(GTKFLAGS) -o $@ $^ $(GTKLIBS) $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~

# There must be a tab at the beginning of any command, and make will not be happy if it's not there.

# *~ means all files ending in ~. Many Unix/Linux systems programs create backup files that end in ~

# INCDIR: include directory parameter
