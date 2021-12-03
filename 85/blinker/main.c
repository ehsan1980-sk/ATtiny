/**
 * main.c
 *
 * Author: Kenta Ishii
 * License: 3-Clause BSD License
 * License URL: https://opensource.org/licenses/BSD-3-Clause
 *
 */

#define F_CPU 8000000UL // Default 8.0Mhz to ATtiny85
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay.h>

int main(void) {
	PORTB = 0; // All Low
	DDRB |= _BV(DDB3); // Bit Value Set (Same as Logical Shift Left) PB3 as Output
	_NOP(); // Wait for Synchronization

	while(1) {
		PORTB ^= _BV(PB3); // Toggle PB3
		_delay_ms(1000);
	}
	return 0;
}
