#
# librfm3.rules.mk
#
# This file is part of the i2c-star project.
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#

LIBRFM3_DIR = ../../librfm3

OBJS += \
	i2c_ctx.o

vpath %.c $(LIBRFM3_DIR)/src

CPPFLAGS += -I$(LIBRFM3_DIR)/include
