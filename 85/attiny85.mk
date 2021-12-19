##
# Copyright 2021 Kenta Ishii
# License: 3-Clause BSD License
# SPDX Short Identifier: BSD-3-Clause
##

# Default Name of Program
NAME ?= program_attiny85

# Location of Folder Headers
HEADER_GLOBAL := ../../
HEADER_LOCAL := ../

# Main C Code
OBJ1 := main

# Library C Code
#OBJ2 := libary

COMP := avr
CC := $(COMP)-gcc
AS := $(COMP)-as
LINKER := $(COMP)-ld
COPY := $(COMP)-objcopy
DUMP := $(COMP)-objdump

ARCH := avr25
MCU  := attiny85
# Programmer
PROG ?= linuxgpio
INTERVAL ?= 100
EFUSE ?= 0xFF
HFUSE ?= 0xDF
# Unprogrammed CKDIV8, 8.0MHz Clock
LFUSE ?= 0xE2
# Default Calibration of Internal RC Oscillator for Individual Difference, Operating Voltage, and Temperature
CALIB_VALUE ?= 0x00
CALIB_DEFINE := -DCALIB_OSCCAL=$(CALIB_VALUE)

# "$@" means the target and $^ means all of dependencies and $< is first one.
# If you meets "make: `main' is up to date.", use "touch" command to renew.
# "$?" means ones which are newer than the target.
# Make sure to use tab in command line

# Make Hex File (Main Target) and Disassembled Dump File
.PHONY: all
all: $(NAME).hex
$(NAME).hex: $(NAME).elf
	$(COPY) $< $@ -O ihex -R .eeprom
	$(DUMP) -D -m $(ARCH) $< > $(NAME).dump

$(NAME).elf: $(OBJ1).o
	$(LINKER) $^ -o $@ -Map $(NAME).map

# Make Object File from C
$(OBJ1).o: $(OBJ1).c
	$(CC) $< -o $@ -mmcu=$(MCU) -Wall -Os -I$(HEADER_GLOBAL) -I$(HEADER_LOCAL) $(CALIB_DEFINE)

.PHONY: warn
warn: all clean

.PHONY: clean
clean:
	rm $(OBJ1).o $(NAME).elf $(NAME).map $(NAME).hex $(NAME).dump

.PHONY: install
install:
	sudo avrdude -p $(MCU) -c $(PROG) -v -i $(INTERVAL) -U efuse:w:$(EFUSE):m -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m -U flash:w:$(NAME).hex:a
