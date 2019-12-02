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

	PORTB = 0; // All Low
	DDRB |= _BV(DDB1); // Bit Value Set (Same as Logical Shift Left) PB1 (OC0B) as Output
	_NOP(); // Wait for Synchronization

	// Counter Reset
	TCNT0 = 0;
	// Reset Toggle Timing for OC0B
	OCR0B = 0;
	// Select CTC Mode and Toggle Output on OC0B
	TCCR0A = _BV(WGM01)|_BV(COM0B0);
	// Start Counter with I/O-Clock / 256, 37500 Hz to Approx. 73.24 Hz
	TCCR0B = _BV(CS02);

	while(1) {
		// Send Random Value
		srand(random_value - (TCNT0<<8|TCNT0)); // uint16_t
		random_value = rand();
		OCR0B = random_value;
	}
	return 0;
}


