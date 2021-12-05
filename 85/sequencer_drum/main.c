/**
 * Copyright 2021 Kenta Ishii
 * License: 3-Clause BSD License
 * SPDX Short Identifier: BSD-3-Clause
 */

#define F_CPU 6400000UL // 6.4Mhz to ATtiny85
#include <stdlib.h>
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#define CALIB_OSCCAL 0x00 // Frequency Calibration for Individual Difference at VCC = 3.0V
#define VOLTAGE_BIAS 0x80 // Decimal 128 on Noise Off

#define RANDOM_INIT 0x4000 // Initial Value to Making Random Value, Must Be Non-zero
inline void random_make( uint8_t bool_high ); // bool_high: True (Not Zero) = 15-bit LFSR-2 (32767 Cycles), Flase (Zero) = 7-bit LFSR-2 (127 Cycles)
uint16_t random_value;

/**
 * Output from PB0 (OC0A)
 * Input from PB1 (Trigger Bit[0]), Set by Detecting Low
 * Input from PB2 (Trigger Bit[1]), Set by Detecting Low
 * Input from PB3 (Trigger Bit[2]), Set by Detecting Low
 * Trigger Bit[2:0]:
 *     0b000: Stop Sequencer
 *     0b001: Play Sequence No.1
 *     0b010: Play Sequence No.2
 *     0b011: PLay Sequence No.3
 *     0b100: PLay Sequence No.4
 *     ...
 * Button 1 from PB4, Push by Detecting Low to Change Beats per Second
 */

#define SAMPLE_RATE (double)(F_CPU / 256) // 25000 Samples per Seconds
#define SEQUENCER_INTERVAL_NUMBER 8
#define SEQUENCER_INTERVAL_INDEX_DEFAULT 0
#define SEQUENCER_COUNTUPTO 64
#define SEQUENCER_SEQUENCENUMBER 5 // Maximum Number of Sequence
#define INPUT_SENSITIVITY 250 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)
#define BUTTON_SENSITIVITY 2500 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint16_t sequencer_interval_max;
uint16_t sequencer_interval_count;
uint16_t sequencer_count_update;
uint8_t sequencer_volume;

// Interval (25000 Divided by Beats in 1 Second)
uint16_t const array_interval[SEQUENCER_INTERVAL_NUMBER] PROGMEM = { // Array in Program Space
	1563, // 16 Beats
	1667, // 15 Beats
	1786, // 14 Beats
	1923, // 13 Beats
	2083, // 12 Beats
	2273, // 11 Beats
	2500, // 10 Beats
	2778 // 9 Beats
};

// Delay Time in Turns to Generate Next Random Value
uint16_t const array_delay_time[16] PROGMEM = { // Array in Program Space
	0,
	1,
	2,
	4,
	8,
	16,
	32,
	64,
	128,
	256,
	512,
	1024,
	2048,
	4096,
	8192,
	16384,
};

uint8_t const array_volume_mask[8] PROGMEM = { // Array in Program Space
	0x00,
	0x07, // Up to Decimal 7
	0x0F, // Up to Decimal 15
	0x1F, // Up to Decimal 31
	0x3F, // Up to Decimal 63
	0x7F,  // Up to Decimal 127
	0xBF,  // Up to Decimal 191
	0xFF // Up to Decimal 255
};

uint8_t const array_volume_offset[8] PROGMEM = { // Array in Program Space
	VOLTAGE_BIAS,
	0x7C, // Decimal 124
	0x78, // Decimal 120
	0x70, // Decimal 112
	0x60, // Decimal 96
	0x40,  // Decimal 64
	0x20,  // Decimal 32
	0x00,  // Decimal 0
};

/**
 * Bit[3:0]: Index of array_delay_time (0-15)
 * Bit[6:4]: Index of array_volume (0-7)
 * Bit[7]: 0 as 7-bit LFSR-2, 1 as 15-bit LFSR-2
 */
uint8_t const sequencer_array_a[SEQUENCER_SEQUENCENUMBER][SEQUENCER_COUNTUPTO] PROGMEM = { // Array in Program Space
	{0x90,0xA0,0xB0,0xC0,0xD0,0xE0,0xF0,0xF0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
	 0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,
	 0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	 0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80}, // Sequence No.1
	{0x90,0xA0,0xB0,0xC0,0xD0,0xE0,0xF0,0xF0,0xC2,0xC2,0xC2,0xC2,0xC2,0xC2,0xC2,0xC2,
	 0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,0xB4,
	 0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,0xA6,
	 0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x98,0x8A,0x8A,0x8A,0x8A}, // Sequence No.2
	{0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,
	 0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,
	 0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,
	 0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xFF}, // Sequence No.3
	{0x10,0x20,0x30,0x40,0x50,0x60,0x70,0x70,0x40,0x40,0x40,0x40,0x40,0x40,0x40,0x40,
	 0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,
	 0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,
	 0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x00,0x00,0x00}, // Sequence No.4
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // Sequence No.5
};


int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_input = _BV(PINB3)|_BV(PINB2)|_BV(PINB1); // Assign PB3, PB2 and PB1 as Trigger Bit[1:0]
	uint8_t const pin_input_shift = PINB1;
	uint8_t const pin_button_1 = _BV(PINB4);
	uint16_t count_delay;
	uint16_t max_count_delay;
	uint8_t volume_mask;
	uint8_t volume_offset;
	uint8_t noise_cycle;
	uint8_t start_noise;
	uint16_t sequencer_count_last = 0;
	uint8_t input_pin;
	uint8_t input_pin_last = 0;
	uint8_t input_pin_buffer = 0;
	uint8_t input_pin_buffer_last = 0;
	uint8_t sequencer_interval_index = 0;
	uint8_t sequencer_array_a_index = 0;
	uint8_t sequencer_byte;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	uint16_t input_sensitivity_count = INPUT_SENSITIVITY;
	uint16_t button_1_sensitivity_count = BUTTON_SENSITIVITY;

	/* Initialize Global Variables */
	sequencer_interval_max = pgm_read_word(&(array_interval[SEQUENCER_INTERVAL_INDEX_DEFAULT]));
	sequencer_interval_index = SEQUENCER_INTERVAL_INDEX_DEFAULT;
	sequencer_interval_count = 0;
	sequencer_count_update = 0;
	sequencer_volume = VOLTAGE_BIAS;
	random_value = RANDOM_INIT;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.0V
	OSCCAL = osccal_default;

	/* I/O Settings */
	DDRB = _BV(DDB0);
	PORTB = _BV(PB4)|_BV(PB3)|_BV(PB2)|_BV(PB1); // Pullup Button Input (There is No Internal Pulldown)

	/* Counter/Timer */
	// Counter Reset
	TCNT0 = 0;
	// Set Output Compare A
	OCR0A = VOLTAGE_BIAS;
	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK = _BV(TOIE0);
	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	// Start Counter with I/O-Clock 6.4MHz / ( 1 * 256 ) = 25000Hz
	TCCR0B = _BV(CS00);
	// Initialize Local Variables Before Loop
	count_delay = 0;
	max_count_delay = 0;
	volume_mask = 0x00;
	volume_offset = VOLTAGE_BIAS;
	noise_cycle = 0;
	start_noise = 0;

	while(1) {
		input_pin = ((PINB ^ pin_input) & pin_input) >> pin_input_shift;
		if ( input_pin >= SEQUENCER_SEQUENCENUMBER ) input_pin = SEQUENCER_SEQUENCENUMBER;
		if ( input_pin == input_pin_last ) { // If Match
			if ( ! --input_sensitivity_count ) { // If Count Reaches Zero
				input_pin_buffer = input_pin;
				input_sensitivity_count = INPUT_SENSITIVITY;
			}
		} else { // If Not Match
			input_pin_last = input_pin;
			input_sensitivity_count = INPUT_SENSITIVITY;
		}
		if ( input_pin_buffer != input_pin_buffer_last ) {
			input_pin_buffer_last = input_pin_buffer;
			if ( input_pin_buffer_last ) { // If Not Zero
				sequencer_array_a_index = input_pin_buffer_last - 1;
				sequencer_interval_count = 0;
				sequencer_count_update = 1;
				sequencer_count_last = 0;
				random_value = RANDOM_INIT; // Reset Random Value
				TIFR |= _BV(TOV0); // Clear Set Timer/Counter0 Overflow Flag by Logic One
				if ( ! start_noise ) start_noise = 1;
				if ( ! (SREG & _BV(SREG_I)) ) sei(); // If Global Interrupt Enable Flag Is Not Set, Start to Issue Interrupt
			}
		}
		if ( sequencer_count_update != sequencer_count_last ) {
			if ( sequencer_count_update > SEQUENCER_COUNTUPTO ) { // If Count Reaches Last
				sequencer_count_update = SEQUENCER_COUNTUPTO + 1;
				cli(); // Stop to Issue Interrupt
			}
			sequencer_count_last = sequencer_count_update;
			if ( sequencer_count_last <= SEQUENCER_COUNTUPTO ) {
				sequencer_byte = pgm_read_byte(&(sequencer_array_a[sequencer_array_a_index][sequencer_count_last - 1]));
				max_count_delay = pgm_read_word(&(array_delay_time[sequencer_byte & 0xF]));
				volume_mask = pgm_read_byte(&(array_volume_mask[(sequencer_byte & 0x70) >> 4]));
				volume_offset = pgm_read_byte(&(array_volume_offset[(sequencer_byte & 0x70) >> 4]));
				noise_cycle = sequencer_byte & 0x80;
			} else {
				max_count_delay = 0;
				volume_mask = 0x00;
				volume_offset = VOLTAGE_BIAS;
				noise_cycle = 0;
				start_noise = 0;
			}
		}
		if ( (PINB ^ pin_button_1) & pin_button_1 ) { // If Match
			if ( ! --button_1_sensitivity_count ) { // If Count Reaches Zero
				button_1_sensitivity_count = BUTTON_SENSITIVITY;
				if ( ++sequencer_interval_index >= SEQUENCER_INTERVAL_NUMBER ) sequencer_interval_index = 0;
				sequencer_interval_max = pgm_read_word(&(array_interval[sequencer_interval_index]));
			}
		} else { // If Not Match
			button_1_sensitivity_count = BUTTON_SENSITIVITY;
		}
		if ( count_delay > max_count_delay ) {
			if ( start_noise ) random_make( noise_cycle );
			count_delay = 0;
			OCR0A = (random_value & volume_mask) + volume_offset;
		}
		count_delay++;
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	sequencer_interval_count++;
	if ( sequencer_interval_count >= sequencer_interval_max ) {
		sequencer_interval_count = 0;
		sequencer_count_update++;
	}
}

inline void random_make( uint8_t bool_high ) { // The inline attribute doesn't make a call, but implants codes.
	random_value = (random_value >> 1)|((((random_value & 0x2) >> 1)^(random_value & 0x1)) << (bool_high ? 14 : 6));
}
