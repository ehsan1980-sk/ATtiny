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

/**
 * Output Sawtooth Wave from PB0 (OC0A)
 * Output Square Wave from PB1 (OC0B)
 */

#define SAMPLE_RATE (F_CPU / 256) // 37500
uint8_t const voltage_bias = 0x80;
uint8_t const absolute_peak = 0x7F;

uint16_t sample_count = 0; // 0-255
uint16_t frequency = 937; // 937.5 Hz
uint16_t count_per_pi = 19; // Count per Pi (Approx. 3.14) Radian
uint16_t count_per_2pi = 39; // Count per 2Pi Radian

/**
 *                      SAMPLE_RATE
 * count_per_2pi + 1 = -------------
 *                       frequency
 */

uint16_t delta_sawtooth = 0x68; // Fixed Point 6.5, Bit[15:12] Reserved for Calculation, Bit[11:4] UINT8, Bit[3:0] Decimal Place

/**
 *                    255 (Peak to Peak)
 * delta_sawtooth = ----------------------
 *                      count_per_2pi
 */

int main(void) {
	OSCCAL += 0x04; // Frequency Calibration for Individual Difference at VCC = 3.3V

	PORTB = 0; // All Low
	DDRB = 0; // All Input
	PORTB |= _BV(PB4); // Pullup Button Input (There is No Internal Pulldown)
	DDRB |= _BV(DDB1)|_BV(DDB0); // Bit Value Set PB0 (OC0A) and PB1 (OC0B) as Output
	DDRB &= ~(_BV(DDB4)); // Bit Value Clear PB4 as Button Input
	_NOP(); // Wait for Synchronization

	// Counter Reset
	TCNT0 = 0;

	// Set Output Compare A
	OCR0A = voltage_bias;

	// Set Output Compare B
	OCR0B = voltage_bias;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted and OC0B Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0B1)|_BV(COM0A1);

	// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
	TCCR0B = _BV(CS00);

	// Start to Issue Interrupt
	sei(); // cli() for Stop

	while(1) {
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	uint8_t const pin_button = _BV(PINB4); // Assign PB4 as Button Input
	uint16_t temp1;
	uint16_t temp2;

	if ( ! (PINB & pin_button) ) { // Start Function
		// Saw Tooth Wave
		if ( sample_count < count_per_2pi ) {
			temp1 = sample_count << 4; // Make Fixed Point, Bit[11:4] UINT8, Bit[3:0] Decimal Place
			temp2 = (delta_sawtooth * temp1) >> 8; // Fixed Point, Logical Shift Right 8 Times to Make Bit[7:0] UINT8
			if (temp2 > 0xFF) temp2 = 0xFF; // Saturate at 8-bit
			OCR0A = temp2;
		} else {
			OCR0A = voltage_bias + absolute_peak;
		}
		// Square Wave
		if ( sample_count <= count_per_pi ) {
			OCR0B = voltage_bias + absolute_peak;
		} else {
			OCR0B = voltage_bias - absolute_peak;
		}
		sample_count++;
		if ( sample_count > count_per_2pi ) sample_count = 0;
	} else { // Stop Function
		sample_count = 0;
		OCR0A = voltage_bias;
		OCR0B = voltage_bias;
	}
}
