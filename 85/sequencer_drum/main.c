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
#include "sequencer.h"
#include "include/random.h"

#ifndef CALIB_OSCCAL
#define CALIB_OSCCAL 0x00
#warning "CALIB_OSCCAL is defined with the default value 0x00."
#endif

/**
 * PWM Output (OC0A): PB0 (DC Biased)
 * Input Bit[0]: PB1 (Pulled Up, Set by Detecting Low)
 *   0b0: Sequence Index No. 0
 *   0b1: Sequence Index No. 1
 * Button 1: PB2 (Pulled Up), Start or Stop Sequence
 * Button 2: PB3 (Pulled Up), Change Output Level
 * Button 3: PB4 (Pulled Up), Change Beats per Second
 */

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
	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIMER0_OVF_vect)"
	TIMSK = _BV(TOIE0);
	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	// Start Counter with I/O-Clock 8.0MHz / ( 1 * 256 ) = 31250Hz
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
			OCR0A = (uint8_t)((((int16_t)(((uint8_t)(random_high_resolution ? random_value : random_value << 1) & volume_mask) + volume_offset) - SEQUENCER_VOLTAGE_BIAS) >> level_shift) + SEQUENCER_VOLTAGE_BIAS);
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

ISR(TIMER0_OVF_vect) {
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
