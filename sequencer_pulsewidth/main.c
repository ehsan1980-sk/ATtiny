/**
 * main.c
 *
 * Author: Kenta Ishii
 * License: 3-Clause BSD License
 * License URL: https://opensource.org/licenses/BSD-3-Clause
 *
 */

#define F_CPU 9600000UL // Default 9.6Mhz to ATtiny13
#include <stdlib.h>
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#define CALIB_OSCCAL 0x03 // Frequency Calibration for Individual Difference at VCC = 3.3V

/**
 * Output from PB0 (OC0A)
 * Output from PB1 (OC0B)
 * Input from PB2 (Bit[0]), Set by Detecting Low
 * Input from PB3 (Bit[1]), Set by Detecting Low
 * Input from PB4 (Bit[2]), Set by Detecting Low
 * Bit[2:0]:
 *     0b000: Stop Sequencer
 *     0b001: Play Sequence No.1
 *     0b010: Play Sequence No.2
 *     0b011: PLay Sequence No.3
 *     0b100: PLay Sequence No.4
 *     ...
 */

#define SAMPLE_RATE (double)(F_CPU / 510 * 64) // Approx. 294.117647 Samples per Seconds
#define SEQUENCER_INTERVAL 30 // Approx. 8Hz = 0.102 Seconds
#define SEQUENCER_COUNTUPTO 64 // 0.102 Seconds * 64
#define SEQUENCER_SEQUENCENUMBER 4 // Miximum Number of Sequence

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint8_t sequencer_count_start;
uint16_t sequencer_interval_count;
uint16_t sequencer_count_update;

/**
 * Sequences for OC0A
 * Bit[7:0]: 0-255 Pulse Width Select
 */
uint8_t const sequencer_array_a[SEQUENCER_SEQUENCENUMBER][SEQUENCER_COUNTUPTO] PROGMEM = { // Array in Program Space
	{ 10, 30, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150,160,
	   0,  0,  0,  0,  0,255,255,255,255,  0,  0,  0,  0,127,127,127,
	 255,255,  0,  0,  0,127,127,  0,  0,  0,  0,127,127,255,255,255,
	  10, 30, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150,160}, // Sequence No.1
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, // Sequence No.2
	{127,238,237,236,235,234,233,232,231,232,233,234,235,236,237,238,
	 239,240,241,242,243,244,245,246,247,246,245,244,243,242,241,240,
	 127,238,237,236,235,234,233,232,231,232,233,234,235,236,237,238,
	 239,240,241,242,243,244,245,246,247,246,245,244,243,242,241,240}, // Sequence No.3
	{255,240,230,220,210,200,190,180,170,160,150,140,130,120,110,100,
	  90, 80, 70, 60, 50, 40, 30, 20, 10,  0,  0,  0,  0,127,127,127,
	 255,255,  0,  0,  0,127,127,  0,  0,  0,  0,127,127,255,255,255,
	  10, 30, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150,160}  // Sequence No.4
};

/**
 * Sequences for OC0B
 * Bit[7:0]: 0-255 Pulse Width Select
 */
uint8_t const sequencer_array_b[SEQUENCER_SEQUENCENUMBER][SEQUENCER_COUNTUPTO] PROGMEM = { // Array in Program Space
	{ 10, 30, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150,160,
	   0,  0,  0,  0,  0,255,255,255,255,  0,  0,  0,  0,127,127,127,
	 255,255,  0,  0,  0,127,127,  0,  0,  0,  0,127,127,255,255,255,
	  10, 30, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150,160}, // Sequence No.1
	{  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
	 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	 255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,
	   0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, // Sequence No.2
	{127,238,237,236,235,234,233,232,231,232,233,234,235,236,237,238,
	 239,240,241,242,243,244,245,246,247,246,245,244,243,242,241,240,
	 127,238,237,236,235,234,233,232,231,232,233,234,235,236,237,238,
	 239,240,241,242,243,244,245,246,247,246,245,244,243,242,241,240}, // Sequence No.3
	{ 10, 30, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150,160,
	   0,  0,  0,  0,  0,255,255,255,255,  0,  0,  0,  0,127,127,127,
	 255,255,  0,  0,  0,127,127,  0,  0,  0,  0,127,127,255,255,255,
	  10, 30, 30, 40, 50, 60, 70, 80, 90,100,110,120,130,140,150,160}  // Sequence No.4
};

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_button1 = _BV(PINB2); // Assign PB2 as Button Input
	uint8_t const pin_button2 = _BV(PINB3); // Assign PB3 as Button Input
	uint8_t const pin_button3 = _BV(PINB4); // Assign PB4 as Button Input
	uint16_t sequencer_count_last = 0;
	uint8_t input_pin;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL

	/* Initialize Global Variables */

	sequencer_count_start = 0;
	sequencer_interval_count = 0;
	sequencer_count_update = 0;
	sequencer_count_last = 0;

	/* Clock Calibration */

	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */

	DIDR0 = _BV(PB5)|_BV(PB1)|_BV(PB0); // Digital Input Disable
	PORTB = _BV(PB4)|_BV(PB3)|_BV(PB2); // Pullup Button Input (There is No Internal Pulldown)
	DDRB = _BV(DDB1)|_BV(DDB0); // Bit Value Set PB0 (OC0A) and PB1 (OC0B)

	/* Counter/Timer */

	// Counter Reset
	TCNT0 = 0;

	// Clear Compare A
	OCR0A = 0;

	// Clear Compare B
	OCR0B = 0;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select PWM (Phase Correct) Mode (1) and Output from OC0A Non-inverted and OC0B Non-inverted
	TCCR0A = _BV(WGM00)|_BV(COM0B1)|_BV(COM0A1);

	// Start Counter with I/O-Clock 9.6MHz / ( 510 * 64 ) = Approx. 294.117647Hz
	TCCR0B = _BV(CS00)|_BV(CS01);

	while(1) {
		input_pin = 0;
		if ( ! (PINB & pin_button1) ) {
			input_pin |= 0b01;
		}
		if ( ! (PINB & pin_button2) ) {
			input_pin |= 0b10;
		}
		if ( ! (PINB & pin_button3) ) {
			input_pin |= 0b100;
		}
		if ( input_pin ) {
			if ( ! sequencer_count_start || sequencer_count_update != sequencer_count_last ) {
				if ( ! sequencer_count_start ) {
					TCNT0 = 0; // Counter Reset
					TIFR0 |= _BV(TOV0); // Clear Set Timer/Counter0 Overflow Flag by Logic One
					sequencer_count_start = 1;
					sei(); // Start to Issue Interrupt
				}
				if ( sequencer_count_update >= SEQUENCER_COUNTUPTO ) sequencer_count_update = 0;
				if ( input_pin >= SEQUENCER_SEQUENCENUMBER ) input_pin = SEQUENCER_SEQUENCENUMBER;
				OCR0A = pgm_read_byte(&(sequencer_array_a[input_pin - 1][sequencer_count_update]));
				OCR0B = pgm_read_byte(&(sequencer_array_b[input_pin - 1][sequencer_count_update]));
				sequencer_count_last = sequencer_count_update;
			}
		} else {
			if ( SREG & _BV(SREG_I) ) { // If Global Interrupt Enable Flag Is Set
				cli(); // Stop to Issue Interrupt
				sequencer_count_start = 0;
				sequencer_interval_count = 0;
				sequencer_count_update = 0;
				sequencer_count_last = 0;
				OCR0A = 0;
				OCR0B = 0;
			}
		}
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	if ( sequencer_count_start ) { // If Not Zero, Sequencer Is Outstanding
		sequencer_interval_count++;
		if ( sequencer_interval_count >= SEQUENCER_INTERVAL ) {
			sequencer_interval_count = 0;
			sequencer_count_update++;
		}
	}
}
