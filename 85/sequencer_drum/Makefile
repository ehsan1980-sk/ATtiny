##
# Copyright 2021 Kenta Ishii
# License: 3-Clause BSD License
# SPDX Short Identifier: BSD-3-Clause
##

COMP := avr
CC := $(COMP)-gcc
COPY := $(COMP)-objcopy
DUMP := $(COMP)-objdump
OBJ1 := main
ARCH := avr25
MCU  := attiny85
# Programmer
PROG := linuxgpio
INTERVAL := 100
EFUSE := 0xFF
HFUSE := 0xDF
# Unprogrammed CKDIV8, Internal 8.0MHz Clock
LFUSE := 0xE2

# "$@" means the target and $^ means all of dependencies and $< is first one.
# If you meets "make: `main' is up to date.", use "touch" command to renew.
# "$?" means ones which are newer than the target.
# Make sure to use tab in command line

# Make Hex File (Main Target) and Disassembled Dump File
.PHONY: all
all: $(OBJ1).hex
$(OBJ1).hex: $(OBJ1).out
	$(COPY) $< $@ -O ihex -R .eeprom
	$(DUMP) -D -m $(ARCH) $< > $(OBJ1).dump

# Make Object File
$(OBJ1).out: $(OBJ1).c
	$(CC) $< -o $@ -mmcu=$(MCU) -Wall -Os

.PHONY: warn
warn: all clean

.PHONY: clean
clean:
	rm $(OBJ1).out $(OBJ1).hex $(OBJ1).dump

.PHONY: install
install:
	sudo avrdude -p $(MCU) -c $(PROG) -v -i $(INTERVAL) -U efuse:w:$(EFUSE):m -U hfuse:w:$(HFUSE):m -U lfuse:w:$(LFUSE):m -U flash:w:$(OBJ1).hex:a
