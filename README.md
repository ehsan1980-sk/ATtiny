# TinyAVR/ATtiny

## Purpose

* To Develop Application of TinyAVR/ATtiny Using C Language

## Installation

* Install Packages to Raspbian Buster on Raspberry Pi

```bash
# Programmer
sudo apt-get install avrdude
sudo apt-get install avrdude-doc
# Binary Utilities Such as Assembler
sudo apt-get install binutils-avr
# Compiler and C Library
sudo apt-get install gcc-avr
sudo apt-get install avr-libc
```

* Open /etc/avrdude.conf

```bash
sudo nano /etc/avrdude.conf
```

* Uncomment and Modify "linuxgpio" Lines as Follows

```bash
programmer
  id    = "linuxgpio";
  desc  = "Use the Linux sysfs interface to bitbang GPIO lines";
  type  = "linuxgpio";
  reset = 22;
  sck   = 23;
  mosi  = 24;
  miso  = 25;
;
```
