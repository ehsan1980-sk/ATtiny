/**
 * main.c
 *
 * Author: Kenta Ishii
 * License: 3-Clause BSD License
 * License URL: https://opensource.org/licenses/BSD-3-Clause
 *
 */

#define F_CPU 9600000UL // Default 9.6Mhz to ATtiny13
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <util/delay.h>

void software_uart_init( uint8_t portb_pin_number );
void software_uart_tx_9600( uint8_t character, uint8_t portb_pin_number );

int main(void) {
	software_uart_init( 3 );

	while(1) {
		software_uart_tx_9600( 0x35, 3 );
		_delay_ms( 1000 );
	}
	return 0;
}

void software_uart_init( uint8_t portb_pin_number ) {
	PORTB |= _BV( portb_pin_number );
	DDRB |= _BV( portb_pin_number );
	_NOP();
}

void software_uart_tx_9600( uint8_t character, uint8_t portb_pin_number ) {
	int8_t set_bit_pin = _BV( portb_pin_number );
	int8_t clear_bit_pin = ~( set_bit_pin );
	for ( int8_t i = 0; i < 10; i++ ) { // Approx. 104.167 Seconds (1000 Cycles)
		// Max. 15 Cycles
		if ( i == 0 ) { // Start Bit (0)
			_NOP();
			_NOP();
			_NOP();
			_NOP();
			_NOP();
			_NOP();
			PORTB &= clear_bit_pin;
		} else if ( i == 9 ) { // Stop Bit (1)
			_NOP();
			_NOP();
			_NOP();
			_NOP();
			_NOP();
			PORTB |= set_bit_pin;
		} else { // Data from LSB to MSB
			if ( character & 1 ) {
				PORTB |= set_bit_pin;
				_NOP();
			} else {
				_NOP();
				PORTB &= clear_bit_pin;
				_NOP();
			}
			character = character >> 1;
		}
		_delay_us( 102 ); // 102 Microseconds, 980 Cycles
		//_delay_ms( 500 );
		_NOP();
		// Additional 4 Cycles If Loop
	}
}
