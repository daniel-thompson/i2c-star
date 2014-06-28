#
# librfn.rules.mk
# 
# This file is part of the i2c-star project.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#

LIBRFN_DIR = ../../librfn

OBJS += \
	bitops.o \
	fibre.o \
	fibre_default.o \
	list.o \
	messageq.o \
	regdump.o \
	ringbuf.o \
	time_libopencm3.o \
	util.o

vpath %.c $(LIBRFN_DIR)/librfn
vpath %.c $(LIBRFN_DIR)/librfn/libopencm3

CPPFLAGS += -DNDEBUG
CPPFLAGS += -I$(LIBRFN_DIR)/include
