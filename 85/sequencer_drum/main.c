/**
 * Copyright 2021 Kenta Ishii
 * License: 3-Clause BSD License
 * SPDX Short Identifier: BSD-3-Clause
 */

#define F_CPU 8000000UL // 8.0Mhz to ATtiny85
#include <stdlib.h>
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#define CALIB_OSCCAL 0x00 // Frequency Calibration for Individual Difference at VCC = 3.0V

/**
 * Output from PB0 (OC0A)
 * Input from PB1 (Trigger Bit[0]), Set by Detecting Low
 * Trigger Bit[0]:
 *     0b0: Sequence Index No. 0
 *     0b1: Sequence Index No. 1
 *
 * Button 1 from PB2, Start/Stop Sequence
 * Button 2 from PB2, Change Output Level
 * Button 3 from PB4, Push by Detecting Low to Change Beats per Second
 */

#define RANDOM_INIT 0x4000 // Initial Value to Making Random Value, Must Be Non-zero
inline void random_make( uint8_t high_resolution ); // high_resolution: True (Not Zero) = 15-bit LFSR-2 (32767 Cycles), Flase (Zero) = 7-bit LFSR-2 (127 Cycles)
uint16_t random_value;

#define SEQUENCER_VOLTAGE_BIAS 0x80 // Decimal 128 on Noise Off
#define SEQUENCER_SAMPLE_RATE (double)(F_CPU / 256) // 31250 Samples per Seconds
#define SEQUENCER_INTERVAL_NUMBER 9
#define SEQUENCER_INTERVAL_INDEX_DEFAULT 0
#define SEQUENCER_PROGRAM_COUNTUPTO 64
#define SEQUENCER_PROGRAM_LENGTH 2 // Length of Sequence
#define SEQUENCER_LEVEL_SHIFT_MAX 3
#define SEQUENCER_INPUT_SENSITIVITY 250 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)
#define SEQUENCER_BUTTON_SENSITIVITY 2500 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint16_t sequencer_interval_max;
uint16_t sequencer_interval_count;
uint16_t sequencer_count_update;
uint16_t sequencer_interval_random;
uint16_t sequencer_interval_random_max;
uint8_t sequencer_next_random;

// Interval (31250 Divided by Beats in 1 Second)
uint16_t const sequencer_interval_array[SEQUENCER_INTERVAL_NUMBER] PROGMEM = { // Array in Program Space
	3906, // 8 Beats
	3472, // 9 Beats
	3125, // 10 Beats
	2841, // 11 Beats
	2604, // 12 Beats
	2404, // 13 Beats
	2232, // 14 Beats
	2083, // 15 Beats
	1953 // 16 Beats
};

// Delay Time in Turns to Generate Next Random Value
uint16_t const sequencer_interval_random_max_array[16] PROGMEM = { // Array in Program Space
	1,
	2,
	3,
	4,
	8,
	16,
	32,
	64,
	128,
	192,
	256,
	384,
	512,
	768,
	1024,
	1536
};

uint8_t const sequencer_volume_mask_array[8] PROGMEM = { // Array in Program Space
	0x00,
	0x07, // Up to Decimal 7
	0x0F, // Up to Decimal 15
	0x1F, // Up to Decimal 31
	0x3F, // Up to Decimal 63
	0x7F, // Up to Decimal 127
	0xBF, // Up to Decimal 191
	0xFF // Up to Decimal 255
};

uint8_t const sequencer_volume_offset_array[8] PROGMEM = { // Array in Program Space
	SEQUENCER_VOLTAGE_BIAS,
	0x7C, // Decimal 124
	0x78, // Decimal 120
	0x70, // Decimal 112
	0x60, // Decimal 96
	0x40, // Decimal 64
	0x20, // Decimal 32
	0x00 // Decimal 0
};

/**
 * Bit[3:0]: Index of sequencer_interval_random_max_array (0-15)
 * Bit[6:4]: Index of sequencer_volume_mask_array and sequencer_volume_offset_array (0-7)
 * Bit[7]: 0 as 7-bit LFSR-2, 1 as 15-bit LFSR-2
 */
uint8_t const sequencer_program_array[SEQUENCER_PROGRAM_LENGTH][SEQUENCER_PROGRAM_COUNTUPTO] PROGMEM = { // Array in Program Space
	{0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xF0,0xF5,0xF0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xF0,0xF5,0xF0}, // Sequence Index No. 0
	{0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,
	 0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x70,0x7F,0x70,
	 0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,
	 0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x30,0x7F,0x3F,0x70,0x70,0x7F,0x70} // Sequence Index No. 1
};

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_input = _BV(PINB1);
	uint8_t const pin_input_shift = PINB1;
	uint8_t const pin_button_1 = _BV(PINB2);
	uint8_t const pin_button_2 = _BV(PINB3);
	uint8_t const pin_button_3 = _BV(PINB4);
	uint8_t volume_mask = 0x00;
	uint8_t volume_offset = SEQUENCER_VOLTAGE_BIAS;
	uint8_t random_high_resolution = 0;
	uint16_t count_last = 0;
	uint8_t input_pin;
	uint8_t input_pin_last = 0;
	uint8_t interval_index = SEQUENCER_INTERVAL_INDEX_DEFAULT;
	uint8_t program_index = 0;
	uint8_t program_byte;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	uint16_t input_sensitivity_count = SEQUENCER_INPUT_SENSITIVITY;
	int16_t button_1_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
	int16_t button_2_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
	int16_t button_3_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
	uint8_t is_start_sequence = 0;
	uint8_t level_shift = 0;

	/* Initialize Global Variables */
	random_value = RANDOM_INIT;
	sequencer_interval_max = pgm_read_word(&(sequencer_interval_array[interval_index]));
	sequencer_interval_count = 0;
	sequencer_count_update = 0;
	sequencer_interval_random = 0;
	sequencer_interval_random_max = 0;
	sequencer_next_random = 0;

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
	OCR0A = SEQUENCER_VOLTAGE_BIAS;
	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK = _BV(TOIE0);
	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	// Start Counter with I/O-Clock 6.4MHz / ( 1 * 256 ) = 25000Hz
	TCCR0B = _BV(CS00);

	while(1) {
		input_pin = ((PINB ^ pin_input) & pin_input) >> pin_input_shift;
		if ( input_pin >= SEQUENCER_PROGRAM_LENGTH ) input_pin = SEQUENCER_PROGRAM_LENGTH - 1;
		if ( input_pin == input_pin_last ) { // If Match
			if ( ! --input_sensitivity_count ) { // If Count Reaches Zero
				program_index = input_pin_last;
				input_sensitivity_count = SEQUENCER_INPUT_SENSITIVITY;
			}
		} else { // If Not Match
			input_pin_last = input_pin;
			input_sensitivity_count = SEQUENCER_INPUT_SENSITIVITY;
		}
		if ( (PINB ^ pin_button_1) & pin_button_1 ) { // If Match
			if ( button_1_sensitivity_count >= 0 ) {
				button_1_sensitivity_count--;
				if ( button_1_sensitivity_count == 0 ) { // If Count Reaches Zero
					if ( ! is_start_sequence ) {
						random_value = RANDOM_INIT; // Reset Random Value
						sequencer_interval_count = 0;
						sequencer_count_update = 1;
						sequencer_interval_random = 0;
						sequencer_interval_random_max = 0;
						count_last = 0;
						TIFR |= _BV(TOV0); // Clear Set Timer/Counter0 Overflow Flag by Logic One
						if ( ! (SREG & _BV(SREG_I)) ) sei(); // If Global Interrupt Enable Flag Is Not Set, Start to Issue Interrupt
						is_start_sequence = 1;
					} else {
						cli(); // Stop to Issue Interrupt
						OCR0A = SEQUENCER_VOLTAGE_BIAS;
						sequencer_next_random = 0;
						is_start_sequence = 0;
					}
				} // If Count Reaches -1, Do Nothing
			}
		} else { // If Not Match
			button_1_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
		}
		if ( sequencer_count_update != count_last ) {
			if ( sequencer_count_update > SEQUENCER_PROGRAM_COUNTUPTO ) { // If Count Reaches Last
				sequencer_count_update = 1;
			}
			count_last = sequencer_count_update;
			program_byte = pgm_read_byte(&(sequencer_program_array[program_index][count_last - 1]));
			sequencer_interval_random_max = pgm_read_word(&(sequencer_interval_random_max_array[program_byte & 0xF]));
			volume_mask = pgm_read_byte(&(sequencer_volume_mask_array[(program_byte & 0x70) >> 4]));
			volume_offset = pgm_read_byte(&(sequencer_volume_offset_array[(program_byte & 0x70) >> 4]));
			random_high_resolution = program_byte & 0x80;
		}
		if ( sequencer_next_random ) {
			random_make( random_high_resolution );
			OCR0A = (uint8_t)((((int16_t)((random_value & volume_mask) + volume_offset) - SEQUENCER_VOLTAGE_BIAS) >> level_shift) + SEQUENCER_VOLTAGE_BIAS);
			sequencer_next_random = 0;
		}
		if ( (PINB ^ pin_button_2) & pin_button_2 ) { // If Match
			if ( button_2_sensitivity_count >= 0 ) {
				button_2_sensitivity_count--;
				if ( button_2_sensitivity_count == 0 ) { // If Count Reaches Zero
					if ( ++level_shift > SEQUENCER_LEVEL_SHIFT_MAX ) level_shift = 0;
				} // If Count Reaches -1, Do Nothing
			}
		} else { // If Not Match
			button_2_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
		}
		if ( (PINB ^ pin_button_3) & pin_button_3 ) { // If Match
			if ( button_3_sensitivity_count >= 0 ) {
				button_3_sensitivity_count--;
				if ( button_3_sensitivity_count == 0 ) { // If Count Reaches Zero
					if ( ++interval_index >= SEQUENCER_INTERVAL_NUMBER ) interval_index = 0;
					sequencer_interval_max = pgm_read_word(&(sequencer_interval_array[interval_index]));
				} // If Count Reaches -1, Do Nothing
			}
		} else { // If Not Match
			button_3_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
		}
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	sequencer_interval_count++;
	if ( sequencer_interval_count >= sequencer_interval_max ) {
		sequencer_interval_count = 0;
		sequencer_count_update++;
	}
	if ( ++sequencer_interval_random >= sequencer_interval_random_max ) {
		sequencer_interval_random = 0;
		sequencer_next_random = 1;
	}
}

inline void random_make( uint8_t high_resolution ) { // The inline attribute doesn't make a call, but implants codes.
	random_value = (random_value >> 1)|((((random_value & 0x2) >> 1)^(random_value & 0x1)) << (high_resolution ? 14 : 6));
}
