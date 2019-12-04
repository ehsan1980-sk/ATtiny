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

#define SAMPLE_RATE (F_CPU / 256) // 37500Hz
uint16_t sample_count = 0;
uint16_t frequency = 1250;
uint16_t count_per_pi = 14; // Count per Pi (Approx. 3.14) Radian
uint16_t count_per_2pi = 29; // Count per 2Pi Radian

/**
 *                      Sample Rate
 * count_per_2pi + 1 = -------------
 *                       Freauency
 */

uint8_t const voltage_bias = 0x80;
uint8_t const absolute_peak = 0x7F;

int main(void) {
	PORTB = 0; // All Low
	DDRB = 0; // All Input
	PORTB |= _BV(PB4); // Pullup Button Input (There is No Internal Pulldown)
	DDRB |= _BV(DDB1); // Bit Value Set PB1 (OC0B) as Output
	DDRB &= ~(_BV(DDB4)); // Bit Value Clear PB4 as Button Input
	_NOP(); // Wait for Synchronization

	// Counter Reset
	TCNT0 = 0;

	// Set Output Compare B
	OCR0B = voltage_bias;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select Fast PWM Mode (3) and Output from 0C0B Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0B1);

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

	if ( ! (PINB & pin_button) ) { // Start Function
		sample_count++;
		// Square
		if ( sample_count <= count_per_pi ) {
			OCR0B = voltage_bias + absolute_peak;
		} else {
			OCR0B = voltage_bias - absolute_peak;
		}
		if ( sample_count == count_per_2pi ) sample_count = 0;
	} else { // Stop Function
		sample_count = 0;
		OCR0B = voltage_bias;
	}
}
