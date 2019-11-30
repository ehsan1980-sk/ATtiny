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
#include <util/delay_basic.h>

void software_uart_init( uint8_t portb_pin_number );
void software_uart_print_9600( char* string, uint16_t length );
void software_uart_tx_9600( uint8_t character, uint8_t portb_pin_number );
uint8_t software_uart_tx_pin;

int main(void) {
	software_uart_init( 3 );

	while(1) {
		software_uart_tx_9600( 0x35, 3 );
		software_uart_print_9600( "Hello World!\r\n", 14 );
		_delay_ms( 1000 );
	}
	return 0;
}

void software_uart_init( uint8_t portb_pin_number ) {
	PORTB |= _BV( portb_pin_number );
	DDRB |= _BV( portb_pin_number );
	_NOP();
	software_uart_tx_pin = portb_pin_number;
}

void software_uart_print_9600( char* string, uint16_t length ) {
	for( uint8_t i = 0; i < length; i++ ) software_uart_tx_9600( string[i], software_uart_tx_pin );
}

void software_uart_tx_9600( uint8_t character, uint8_t portb_pin_number ) {
	/**
	 * First Argument is r24, Second Argument is r22 (r25 to r8, Assign Even Number Registers)
	 * r18-r27, r30-r31 are scratch registers.
	 * r2-r17, r28-r29 need push/pop in the function.
	 * r0 is temporary and r1 is zero.
	 */
	register const uint8_t count_delay asm("r18") = 247; // 247 * 4, 988 Clocks
	register uint8_t count asm("r19");
	register uint8_t set_bit_pin asm("r20") = _BV( portb_pin_number );
	register uint8_t clear_bit_pin asm("r21") = ~( set_bit_pin );;
	register uint8_t i asm("r22") = 9;
	asm volatile (
			"clc" "\n\t" // Clear Carry Bit for Start Bit
		"software_uart_tx_9600_forloop:" "\n\t" // 12 Clocks + Delay 988 Clocks
			"in __tmp_reg__, %[portb]" "\n\t"
			"brcc software_uart_tx_9600_forloop_carryclear" "\n\t" // From Result of lsr
			"or __tmp_reg__, %[set_bit_pin]" "\n\t"
			"rjmp software_uart_tx_9600_forloop_common" "\n\t"
			"software_uart_tx_9600_forloop_carryclear:" "\n\t"
				"and __tmp_reg__, %[clear_bit_pin]" "\n\t"
				"nop" "\n\t"
			"software_uart_tx_9600_forloop_common:" "\n\t"
				"out %[portb], __tmp_reg__" "\n\t"
				// count_delay * 4 Cycles
				"mov %[count], %[count_delay]" "\n\t"
				"software_uart_tx_9600_forloop_common_delay:" "\n\t"
					"nop" "\n\t"
					"subi %[count], 0x1" "\n\t"
					"brne software_uart_tx_9600_forloop_common_delay" "\n\t"
				"subi %[i], 0x1" "\n\t"
				"breq software_uart_tx_9600_stopbit" "\n\t" // Branch If Lower
				"nop" "\n\t"
				"lsr %[character]" "\n\t"
				"rjmp software_uart_tx_9600_forloop" "\n\t"
		"software_uart_tx_9600_stopbit:" "\n\t"
			"in __tmp_reg__, %[portb]" "\n\t"
			"or __tmp_reg__, %[set_bit_pin]" "\n\t"
			"rjmp .+0" "\n\t" // Two Cycles
			"rjmp .+0" "\n\t" // Two Cycles
			"rjmp .+0" "\n\t" // Two Cycles
			"out %[portb], __tmp_reg__" "\n\t"
			//count_delay * 4 Cycles
			"mov %[count], %[count_delay]" "\n\t"
			"inc %[count]" "\n\t"
			"software_uart_tx_9600_stopbit_delay:" "\n\t"
				"nop" "\n\t"
				"subi %[count], 0x1" "\n\t"
				"brne software_uart_tx_9600_stopbit_delay" "\n\t"
				"rjmp .+0" "\n\t" // Two Cycles
		/* Outputs */
		:[count]"=d"(count),
		 [i]"+d"(i)
		/* Inputs */
		:[portb]"I"(_SFR_IO_ADDR(PORTB)),
		 [set_bit_pin]"r"(set_bit_pin),
		 [clear_bit_pin]"r"(clear_bit_pin),
		 [count_delay]"r"(count_delay),
		 [character]"r"(character)
		/* Clobber List */
		:
	);
}
