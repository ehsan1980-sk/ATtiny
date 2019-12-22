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
 * Output from PB0
 * Output from PB1
 * Output from PB2
 * Input from PB3 (Bit[0]), Set by Detecting Low
 * Input from PB4 (Bit[1]), Set by Detecting Low
 * Bit[1:0]:
 *     0b00: Stop Sequencer
 *     0b01: Play Sequence No.1
 *     0b10: Play Sequence No.2
 *     0b11: PLay Sequence No.3
 */

#define SAMPLE_RATE (double)(F_CPU / (256 * 64)) // 585.9375 Samples per Seconds
#define SEQUENCER_INTERVAL 586 // Approx. 1Hz = 1 Seconds
#define SEQUENCER_COUNTUPTO 64 // 1 Seconds * 64
#define SEQUENCER_SEQUENCENUMBER 3 // Miximum Number of Sequence

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint8_t sequencer_count_start;
uint16_t sequencer_interval_count;
uint16_t sequencer_count_update;

/**
 * Sequences for GPIO
 * Bit[2]: PB2
 * Bit[1]: PB1
 * Bit[0]: PB0
 */
uint8_t const sequencer_array[SEQUENCER_SEQUENCENUMBER][SEQUENCER_COUNTUPTO] PROGMEM = { // Array in Program Space
	{0b000,0b001,0b010,0b011,0b100,0b101,0b110,0b111,0b110,0b101,0b100,0b011,0b010,0b001,0b000,0b000,
	 0b000,0b001,0b010,0b011,0b100,0b101,0b110,0b111,0b110,0b101,0b100,0b011,0b010,0b001,0b000,0b000,
	 0b000,0b001,0b010,0b011,0b100,0b101,0b110,0b111,0b110,0b101,0b100,0b011,0b010,0b001,0b000,0b000,
	 0b000,0b001,0b010,0b011,0b100,0b101,0b110,0b111,0b110,0b101,0b100,0b011,0b010,0b001,0b000,0b000}, // Sequence No.1
	{0b100,0b010,0b001,0b010,0b100,0b010,0b001,0b000,0b111,0b000,0b111,0b000,0b111,0b000,0b111,0b000,
	 0b100,0b010,0b001,0b010,0b100,0b010,0b001,0b000,0b111,0b000,0b111,0b000,0b111,0b000,0b111,0b000,
	 0b100,0b010,0b001,0b010,0b100,0b010,0b001,0b000,0b111,0b000,0b111,0b000,0b111,0b000,0b111,0b000,
	 0b100,0b010,0b001,0b010,0b100,0b010,0b001,0b000,0b111,0b000,0b111,0b000,0b111,0b000,0b111,0b000}, // Sequence No.2
	{0b001,0b001,0b000,0b000,0b001,0b001,0b000,0b000,0b001,0b001,0b000,0b000,0b001,0b001,0b000,0b000,
	 0b010,0b010,0b000,0b000,0b010,0b010,0b000,0b000,0b010,0b010,0b000,0b000,0b010,0b010,0b000,0b000,
	 0b100,0b100,0b000,0b000,0b100,0b100,0b000,0b000,0b100,0b100,0b000,0b000,0b100,0b100,0b000,0b000,
	 0b111,0b111,0b000,0b000,0b111,0b111,0b000,0b000,0b001,0b010,0b100,0b111,0b000,0b111,0b000,0b111,}, // Sequence No.3
};

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_button1 = _BV(PINB3); // Assign PB3 as Button Input
	uint8_t const pin_button2 = _BV(PINB4); // Assign PB4 as Button Input
	uint16_t sequencer_count_last = 0;
	uint8_t sequencer_value;
	uint8_t sequencer_output;
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

	DIDR0 = _BV(PB5)|_BV(PB2)|_BV(PB1)|_BV(PB0); // Digital Input Disable
	PORTB = _BV(PB4)|_BV(PB3); // Pullup Button Input (There is No Internal Pulldown)
	DDRB =  _BV(DDB2)|_BV(DDB1)|_BV(DDB0); // Bit Value Set PB0 and PB1

	/* Counter/Timer */

	// Counter Reset
	TCNT0 = 0;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select Normal Mode (0) and No Output
	TCCR0A = 0;

	// Start Counter with I/O-Clock 9.6MHz / ( 256 * 64 ) = 585.9375Hz
	TCCR0B = _BV(CS00)|_BV(CS01);

	while(1) {
		input_pin = 0;
		if ( ! (PINB & pin_button1) ) {
			input_pin |= 0b01;
		}
		if ( ! (PINB & pin_button2) ) {
			input_pin |= 0b10;
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
				sequencer_value = pgm_read_byte(&(sequencer_array[input_pin - 1][sequencer_count_update]));
				sequencer_output = PORTB;
				if ( sequencer_value & _BV(PB0) ) {
					sequencer_output |= _BV(PB0);
				} else {
					sequencer_output &= ~(_BV(PB0));
				}
				if ( sequencer_value & _BV(PB1) ) {
					sequencer_output |= _BV(PB1);
				} else {
					sequencer_output &= ~(_BV(PB1));
				}
				if ( sequencer_value & _BV(PB2) ) {
					sequencer_output |= _BV(PB2);
				} else {
					sequencer_output &= ~(_BV(PB2));
				}
				PORTB = sequencer_output;
				sequencer_count_last = sequencer_count_update;
			}
		} else {
			if ( SREG & _BV(SREG_I) ) { // If Global Interrupt Enable Flag Is Set
				cli(); // Stop to Issue Interrupt
				sequencer_count_start = 0;
				sequencer_interval_count = 0;
				sequencer_count_update = 0;
				sequencer_count_last = 0;
				sequencer_output = PORTB;
				sequencer_output &= ~(_BV(PB2)|_BV(PB1)|_BV(PB0));
				PORTB = sequencer_output;
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
