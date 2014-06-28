##
## This file is part of the i2c-star project
##
## Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
##
## This library is free software: you can redistribute it and/or modify
## it under the terms of the GNU Lesser General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This library is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU Lesser General Public License for more details.
##
## You should have received a copy of the GNU Lesser General Public License
## along with this library.  If not, see <http://www.gnu.org/licenses/>.
##

BINARY = i2c-stm32f4-usb

DEFS += -DF_CPU=120000000
DEFS += -DF_APB1=30000000

LDSCRIPT = ../stm32f4-discovery.ld
include ../../mk/libopencm3.stm32f4xx.mk
