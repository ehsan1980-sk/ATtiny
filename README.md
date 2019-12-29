# TinyAVR/ATtiny

### Information of this README and comments in this project may be incorrect. This project is not an official document of Microchip Technology Inc., Atmel Corporation, and other holders of any Intellectual Property (IP).

## Purpose

* To Develop Software of TinyAVR/ATtiny Using C Language and Assembler Language

**About TinyAVR/ATtiny**

* TinyAVR/ATtiny is a family of microcontrollers. Specially, I'm trying to make software of ATtiny13 which has 8 pins.

* ATtiny13 is the simplest one. Available interfaces are PWM, ADC, GPIO, and Comparator. It also has a 8-bit timer/counter, and has a unique 9.6MHz RC oscillator which allows to make a software UART Tx with the baud rate 9600, 19200, 38400, etc.

## Installation

* Install packages to Raspbian Buster on Raspberry Pi.

```bash
# Do as Superuser, Install Git and Make
sudo apt-get update
sudo apt-get install git
sudo apt-get install make
# AVR Programmer
sudo apt-get install avrdude
sudo apt-get install avrdude-doc
# Binary Utilities including GNU Assmebler
sudo apt-get install binutils-avr
# Assembler Compatible with AVRASM32
sudo apt-get install avra
# Compiler and C Library
sudo apt-get install gcc-avr
sudo apt-get install avr-libc
```

* Open /etc/avrdude.conf to enable programming with GPIOs.

```bash
sudo nano /etc/avrdude.conf
```

* Uncomment and modify "linuxgpio" lines as follows. In this case, GPIO22 for RESET of TinyAVR/ATtiny, GPIO23 for SCK, GPIO24 for MOSI, and GPIO25 for MISO.

```
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

* Connect Raspberry Pi and TinyAVR/ATtiny through assigned GPIOs. When connecting assigned GPIOs and TinyAVR/ATtiny pins to program, use appropriate resisters to limit electric current for hardware's safety. DON'T INPUT VOLTAGE OVER 3.3V TO GPIO PIN! OTHERWISE, YOU WILL BE IN DANGER!

* Connect 3.3V Power of Raspberry Pi to VCC of TinyAVR/ATtiny, GND of Raspberry Pi to GND of TinyAVR/ATtiny. NEVER USE 5.0V POWER.

* Clone this project using Git, and compile the code then program the binary to TinyAVR/ATtiny.

```bash
cd ~/Desktop
git clone https://github.com/JimmyKenMerchant/ATtiny.git
# Enter Directory
cd ATtiny/blinker
# Compile
make
# Program Binary to TinyAVR/ATtiny
make install
```

* Place programmed TinyAVR/ATtiny to your circuit.

## Memos on Development

* ATtiny13 has 64 bytes SRAM which enables stack (pop/push) operations. Caution that SRAM is shared by global variables and stack. Many stack operations may cause memory overflow and corrupt global variables. Arrays of constants can be stored in program space using `PROGMEM` attribute with `<avr/pgmspace.h>` library.

## Sequencer, A Music Box

* Sequencer is a music box. Note that Sequencer GPIO or Sequencer Pulse-width which are aiming to light decorations mainly (Sequencer Pulse-width can be modified as a Voice Box through changing to high sampling rate, even though it needs more SRAM just like in ATmega).

* Sequencer has alternative versions such as "main.c.jinglebells".

```bash
cd ATtiny/sequencer
# Change Original Version to Alternative Version
mv main.c main.c.origin
cp main.c.jinglebells main.c
```

* Sequencer emits 90 degrees phase shifted saw tooth wave; because in the ideal behavior, the wave can be transformed to sine wave through omitting all harmonics. Making square wave is easy; however in my experience, it often has noise like resonance after rising or falling edge, causing losses of electric power.

## Technical Notes

**December 1, 2019**

* `cat /proc/cpuinfo | grep 'Model'`: Raspberry Pi 3 Model B Rev 1.2
* `lsb_release -a | grep 'Description'`: Raspbian GNU/Linux 10 (buster)
* `git --version`: git version 2.20.1
* `make --version`: GNU Make 4.2.1 Built for arm-unknown-linux-gnueabihf
* `avr-gcc --version`: avr-gcc (GCC) 5.4.0
* `avr-as --version`: GNU assembler (GNU Binutils) 2.26.20160125
* `avra --version`: AVRA: advanced AVR macro assembler Version 1.3.0 Build 1 (8 May 2010)
* `avrdude -v`: Version 6.3-20171130
* `apt-cache show avr-libc | grep 'Version'`: 1:2.0.0+Atmel3.6.1-2
* Description: Take these commands at the terminal of Raspbian I use for this project.

## Links of References

* [AVR Libc Home Page](http://www.nongnu.org/avr-libc/)

* [AVR Libc Reference Manual](https://www.microchip.com/webdoc/AVRLibcReferenceManual/index.html)

* [AVR Assembler](https://www.microchip.com/webdoc/GUID-E06F3258-483F-4A7B-B1F8-69933E029363/index.html): Assembler with the original syntax, but not GNU Assembler's syntax. Obtain the documentation of the instruction set at the preface.
