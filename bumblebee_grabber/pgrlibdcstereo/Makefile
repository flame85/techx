
#####################################################################
# Copyright (c) 2008 Point Grey Research Inc.
#
# This Makefile is free software; Point Grey Research Inc.
# gives unlimited permission to copy and/or distribute it,
# with or without modifications, as long as this notice is preserved.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY, to the extent permitted by law; without
# even the implied warranty of MERCHANTABILITY or FITNESS FOR A
# PARTICULAR PURPOSE.
#
#####################################################################

ROOT_INCLUDE = /usr/local/include

# compilation flags
CPPFLAGS += -I.
# libdc1394 installed in /usr/local/include location
CPPFLAGS += -I$(ROOT_INCLUDE)/dc1394
CPPFLAGS += -I$(ROOT_INCLUDE)/triclops
CPPFLAGS += -Wall -g
CPPFLAGS += -DLINUX
#CPPFLAGS += -Wall -O3
DC = libdc1394-2

# compilation flags
CPPFLAGS += `pkg-config --cflags $(DC)`
CPPFLAGS += -Wall -g
CPPFLAGS += -DLINUX
LIBRARY_NAME	= pgrlibdcstereo
LIBRARY		= libmypgrlibdcstereo.a
LIBRARY_SRC	= pgr_conversions.cpp \
		pgr_stereocam.cpp \
		pgr_registers.cpp

all:	$(LIBRARY)

$(LIBRARY): $(LIBRARY_SRC:%.cpp=%.o)
	$(AR) $(ARFLAGS) $@ $^

%.o:%.cpp
	g++ -c $(CXXFLAGS) $(CPPFLAGS) $*.cpp -o $*.o

clean:
	rm -f *~ *.o *.d $(EXECS)

#####################################################################
#
# $Id: Makefile,v 1.2 2008/12/11 00:00:21 warrenm Exp $
# $Author: warrenm $
# $Revision: 1.2 $
# $Date: 2008/12/11 00:00:21 $
#
#####################################################################

