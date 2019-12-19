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
#include <util/delay.h>
#include <util/delay_basic.h>

#define CALIB_OSCCAL 0x03 // Frequency Calibration for Individual Difference at VCC = 3.3V

/**
 * Output Sawtooth Wave from PB0 (OC0A)
 * Input from PB2, Start Seqeuncer If Detected Low
 * Note: The wave may not reach the high peak, 0xFF (255) in default,
 *       because of its low precision decimal system.
 */

#define SAMPLE_RATE (double)(F_CPU / 256) // 37500 Samples per Seconds
#define PEAK_LOW 0x00
#define PEAK_HIGH 0xFF
#define PEAK_TO_PEAK (PEAK_HIGH - PEAK_LOW)
#define SEQUENCER_INTERVAL 9375 // 4Hz = 0.25 Seconds
#define SEQUENCER_COUNTUPTO 10 // 0.25 Seconds * 10

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint8_t osccal_default; // Calibrated Default Value of OSCCAL
int8_t osccal_tuning; // Tuning Value for Variable Tone
uint16_t sample_count; // 0-255
uint16_t count_per_pi; // Count per Pi (Approx. 3.14) Radian
uint16_t count_per_2pi; // Count per 2Pi Radian

/**
 *                      SAMPLE_RATE
 * count_per_2pi + 1 = -------------
 *                       Frequency
 */

uint16_t fixed_value_sawtooth; // Fixed Point Arithmetic, Bit[15] Sign, Bit[14:7] UINT8, Bit[6:0] Fractional Part
uint16_t fixed_delta_sawtooth; // Fixed Point Arithmetic, Bit[15] Sign, Bit[14:7] UINT8, Bit[6:0] Fractional Part
uint16_t fixed_value_triangle; // Fixed Point Arithmetic, Bit[15] Sign, Bit[14:7] UINT8, Bit[6:0] Fractional Part
uint8_t toggle_triangle;

/**
 *                         PEAK_TO_PEAK
 * fixed_delta_sawtooth = ---------------
 *                         count_per_2pi
 */

uint8_t function_start;
uint8_t sequence_start;
uint16_t sequencer_interval_count;
uint16_t seqeuncer_count_update;
uint16_t seqeuncer_count_last;

/**
 * Bit[7:0]: 0-255 Tone Select
 */
uint8_t sequencer_array[] = {
	1,2,3,4,5,6,7,6,7,0
};

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_button = _BV(PINB2); // Assign PB2 as Button Input
	uint16_t count_per_2pi_buffer = 0;
	uint16_t fixed_delta_sawtooth_buffer = 0;
	uint8_t sequencer_value;

	/* Initialize Global Variables */

	osccal_default = OSCCAL;
	osccal_tuning = 0;
	sample_count = 0;
	count_per_pi = 0;
	count_per_2pi = 0;
	fixed_value_sawtooth = 0;
	fixed_delta_sawtooth = 0;
	fixed_value_triangle = 0;
	toggle_triangle = 0;
	sequence_start = 0;
	sequencer_interval_count = 0;
	seqeuncer_count_update = 0;
	seqeuncer_count_last = 0;

	/* Clock Calibration */

	osccal_default += CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */

	PORTB = 0; // All Low
	PORTB |= _BV(PB2); // Pullup Button Input (There is No Internal Pulldown)
	DDRB = 0; // All Input
	DDRB |= _BV(DDB0); // Bit Value Set PB0 (OC0A)

	/* Counter/Timer */

	// Counter Reset
	TCNT0 = 0;

	// Set Output Compare A
	OCR0A = PEAK_LOW;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0A1);

	// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
	TCCR0B = _BV(CS00);

	while(1) {
		if ( ! (PINB & pin_button) ) {
			if ( ! sequence_start ) sequence_start = 1;
			if ( seqeuncer_count_update != seqeuncer_count_last ) {
				sequencer_value = sequencer_array[(seqeuncer_count_update - 1)];
				/* Pentatonic Scale, A Minor and C Major, 37500 Samples per Seconds */
				if ( sequencer_value == 7 ) {
					count_per_2pi_buffer = 35; // C6 1046.50 Hz
					fixed_delta_sawtooth_buffer = 7<<7|0b0100100;
					osccal_tuning = 1;
				} else if ( sequencer_value == 6 ) {
					count_per_2pi_buffer = 41; // A5 880.00 Hz
					fixed_delta_sawtooth_buffer = 6<<7|0b0011100;
					osccal_tuning = -1;
				} else if ( sequencer_value == 5 ) {
					count_per_2pi_buffer = 47; // G5 783.99 Hz
					fixed_delta_sawtooth_buffer = 5<<7|0b0110110;
					osccal_tuning = 1;
				} else if ( sequencer_value == 4 ) {
					count_per_2pi_buffer = 56; // E 659.26
					fixed_delta_sawtooth_buffer = 4<<7|0b1000110;
					osccal_tuning = 1;
				} else if ( sequencer_value == 3 ) {
					count_per_2pi_buffer = 63; // D5 587.33 Hz
					fixed_delta_sawtooth_buffer = 4<<7|0b0000110;
					osccal_tuning = 1;
				} else if ( sequencer_value == 2 ) {
					count_per_2pi_buffer = 70; // C5 523.25 Hz
					fixed_delta_sawtooth_buffer = 3<<7|0b1010010;
					osccal_tuning = 0;
				} else if ( sequencer_value == 1 ) {
					count_per_2pi_buffer = 84; // A4 440.00 Hz
					fixed_delta_sawtooth_buffer = 3<<7|0b0000100;
					osccal_tuning = 0;
				} else {
					count_per_2pi_buffer = 0;
					fixed_delta_sawtooth_buffer = 0;
					osccal_tuning = 0;
				}
				if ( count_per_2pi_buffer != count_per_2pi ) {
					if ( count_per_2pi_buffer ) {
						cli(); // Stop to Issue Interrupt
						sample_count = 0;
						toggle_triangle = 0;
						count_per_2pi = count_per_2pi_buffer;
						count_per_pi = count_per_2pi >> 1;
						fixed_delta_sawtooth = fixed_delta_sawtooth_buffer;
						function_start = 1;
						sei(); // Start to Issue Interrupt
					} else {
						cli(); // Stop to Issue Interrupt
						sample_count = 0;
						toggle_triangle = 0;
						count_per_2pi = 0;
						count_per_pi = 0;
						fixed_delta_sawtooth = 0;
						function_start = 0;
						sei(); // Start to Issue Interrupt
					}
					OSCCAL = osccal_default + osccal_tuning;
				}
				if ( seqeuncer_count_update > SEQUENCER_COUNTUPTO ) {
					seqeuncer_count_update = 0;
				}
				seqeuncer_count_last = seqeuncer_count_update;
			}
		} else {
			cli(); // Stop to Issue Interrupt
			sample_count = 0;
			toggle_triangle = 0;
			count_per_2pi = 0;
			count_per_pi = 0;
			fixed_delta_sawtooth = 0;
			function_start = 0;
			sequence_start = 0;
			sei(); // Start to Issue Interrupt
			sequencer_interval_count = 0;
			seqeuncer_count_update = 0;
			seqeuncer_count_last = 0;
		}
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	uint16_t temp;

	if ( function_start ) { // Start Function
		// Saw Tooth Wave
		if ( sample_count == 0 ) {
			OCR0A = PEAK_LOW;
			fixed_value_sawtooth = PEAK_LOW << 7;
		} else if ( sample_count <= count_per_2pi ) {
			fixed_value_sawtooth += fixed_delta_sawtooth; // Fixed Point Arithmetic (ADD)
			temp = (fixed_value_sawtooth << 1) >> 8; // Make Bit[7:0] UINT8 (Considered of Clock Cycle)
			if ( 0x0040 & fixed_value_sawtooth ) temp++; // Check Fractional Part Bit[6] (0.5) to Round Off
			OCR0A = temp;
		}
		sample_count++;
		if ( sample_count > count_per_2pi ) sample_count = 0;
	} else { // Stop Function
		OCR0A = PEAK_LOW;
	}
	if ( sequence_start ) {
		sequencer_interval_count++;
		if ( sequencer_interval_count >= SEQUENCER_INTERVAL ) {
			sequencer_interval_count = 0;
			seqeuncer_count_update++;
		}
	}
}
