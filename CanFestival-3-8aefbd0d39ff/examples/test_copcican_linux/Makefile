#! gmake

#
# Copyright (C) 2010 Cosateq GmbH & Co.KG
#               http://www.cosateq.com/
#               http://www.scale-rt.com/
#
# This file is part of canfestival, a library implementing the canopen
# stack
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 2.1 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#

CC = gcc
CXX = g++
LD = g++
OPT_CFLAGS = -O2
CFLAGS = $(OPT_CFLAGS)
PROG_CFLAGS =  -DDEBUG_WAR_CONSOLE_ON -DDEBUG_ERR_CONSOLE_ON -DDEBUG_MSG_CONSOLE_ON -g
EXE_CFLAGS =  -lpthread -lrt -ldl
OS_NAME = Linux
ARCH_NAME = x86
PREFIX = /usr/local
TARGET = unix
CAN_DRIVER = can_kvaser
TIMERS_DRIVER = timers_unix

INCLUDES = -I../../include -I../../include/$(TARGET) -I../../include/$(CAN_DRIVER) -I../../include/$(TIMERS_DRIVER)

TEST_COPCICAN_LINUX_OBJS = test_copcican_linux.o

OBJS = $(TEST_COPCICAN_LINUX_OBJS) ../../drivers/$(TARGET)/libcanfestival_$(TARGET).a ../../src/libcanfestival.a

all: test_copcican_linux

../../src/libcanfestival.a:
	$(MAKE) -C ../../src libcanfestival.a


../../drivers/$(TARGET)/libcanfestival_$(TARGET).a:
	$(MAKE) -C ../../drivers/$(TARGET) libcanfestival_$(TARGET).a


test_copcican_linux: test_copcican_linux.c $(OBJS)
	$(LD) $(CFLAGS) $(PROG_CFLAGS) ${PROGDEFINES} $(INCLUDES) -o $@ $(OBJS) $(EXE_CFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) $(PROG_CFLAGS) ${PROGDEFINES} $(INCLUDES) -o $@ -c $<

clean:
	rm -f $(TEST_COPCICAN_LINUX_OBJS)
	rm -f test_copcican_linux

mrproper: clean
	rm -f test_copcican_linux.c

install: test_copcican_linux
	mkdir -p $(PREFIX)/bin/
	cp $< $(PREFIX)/bin/

uninstall:
	rm -f $(PREFIX)/bin/test_copcican_linux


