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
#include <util/delay.h>
#include <util/delay_basic.h>

int main(void) {
	uint16_t random_value = 0xFFFF; // Non-zero Value
	uint8_t pin_button = _BV(PINB4); // Assign PB4 as Button Input

	PORTB = 0; // All Low
	DDRB = 0; // All Input
	PORTB |= _BV(PB4); // Pullup Button Input (There is No Internal Pulldown)
	DDRB |= _BV(DDB1); // Bit Value Set PB1 (OC0B) as Output
	DDRB &= ~(_BV(DDB4)); // Bit Value Clear PB4 as Button Input
	_NOP(); // Wait for Synchronization

	// Counter Reset
	TCNT0 = 0;

	// Select Fast PWM Mode and Output on OC0B
	TCCR0A = _BV(COM0B1)|_BV(WGM01)|_BV(WGM00);
	// Stop Counter
	TCCR0B = 0;

	while(1) {
		if ( ! (PINB & pin_button) ) { // Output Noise
			srand(random_value - (TCNT0<<8|TCNT0)); // uint16_t
			random_value = rand();
			OCR0B = random_value;
			// Start Output
			if ( ! TCCR0B ) {
				// Start Counter with I/O-Clock, 9.6MHz / ( 1 * 256 ) = 37500Hz
				TCCR0B = _BV(CS00);
			}
		} else { // No Output
			// Stop Output
			if ( TCCR0B ) {
				// Stop Counter
				TCCR0B = 0;
			}
		}
	}
	return 0;
}


