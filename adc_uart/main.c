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

/**
 * Send 12-bit 2 Channels as Follows (ADC is 10-bit Resolution)
 * First Byte: Bit[7] Always High, Bit[6:5] Channel Number, Bit[4:0] Most Significant 5 Bits
 * Second Byte: Bit[7] Always Low, Bit[6:0] Least Significant 7 Bits
 */

void software_uart_init( uint8_t portb_pin_number_for_tx );
void software_uart_print_38400( char* string, uint16_t length, uint8_t portb_pin_number_for_tx );
void software_uart_tx_38400( uint8_t character, uint8_t portb_pin_number_for_tx );
uint8_t software_uart_tx_pin;

int main(void) {
	uint8_t const select_adc_channel_1 = _BV(MUX0); // ADC1 (PB2)
	uint8_t const select_adc_channel_2 = _BV(MUX1); // ADC2 (PB4)
	uint8_t const clear_adc_channel = ~(_BV(MUX1)|_BV(MUX0));
	uint8_t const start_adc = _BV(ADSC);
	uint8_t value_adc_channel_1_low; // Bit[7:6] Is ADC[1:0]
	uint8_t value_adc_channel_1_high; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_2_low; // Bit[7:6] Is ADC[1:0]
	uint8_t value_adc_channel_2_high; // Bit[7:0] Is ADC[9:2]

	PORTB = 0; // All Low
	DDRB = 0; // All Input

	software_uart_init( 3 );

	// Set ADC, Vcc as Reference, ADLAR
	ADMUX = _BV(ADLAR);
	// Enable ADC, Prescaler 64 to Have ADC Clock 150Khz.
	// Set more speed for ADC Clock (600Khz), although it affects the absolute resolution. Set ADLAR and get only ADCH for 8 bit resolution.
	ADCSRA = _BV(ADEN)|_BV(ADPS2)|_BV(ADPS1);

	while(1) {
		ADMUX |= select_adc_channel_1;
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		ADMUX &= clear_adc_channel;
		value_adc_channel_1_low = ADCL; // Read Low Bits First
		value_adc_channel_1_high = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read

		ADMUX |= select_adc_channel_2;
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		ADMUX &= clear_adc_channel;
		value_adc_channel_2_low = ADCL; // Read Low Bits First
		value_adc_channel_2_high = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read

		software_uart_tx_38400( 0x80|1<<5|value_adc_channel_1_high>>5, software_uart_tx_pin ); // First Byte for ADC Channel 1
		software_uart_tx_38400( 0x7F&(value_adc_channel_1_high<<2|value_adc_channel_1_low>>6), software_uart_tx_pin ); // Second Byte for ADC Channel 1
		software_uart_tx_38400( 0x80|2<<5|value_adc_channel_2_high>>5, software_uart_tx_pin ); // First Byte for ADC Channel 2
		software_uart_tx_38400( 0x7F&(value_adc_channel_2_high<<2|value_adc_channel_2_low>>6), software_uart_tx_pin ); // Second Byte for ADC Channel 2
		_delay_ms( 500 );
	}
	return 0;
}

void software_uart_init( uint8_t portb_pin_number_for_tx ) {
	PORTB |= _BV( portb_pin_number_for_tx );
	DDRB |= _BV( portb_pin_number_for_tx );
	_NOP();
	software_uart_tx_pin = portb_pin_number_for_tx;
}

void software_uart_print_38400( char* string, uint16_t length, uint8_t portb_pin_number_for_tx ) {
	for( uint16_t i = 0; i < length; i++ ) software_uart_tx_38400( string[i], portb_pin_number_for_tx );
}

void software_uart_tx_38400( uint8_t character, uint8_t portb_pin_number_for_tx ) {
	/**
	 * First Argument is r24, Second Argument is r22 (r25 to r8, Assign Even Number Registers)
	 * r18-r27, r30-r31 are scratch registers.
	 * r2-r17, r28-r29 need push/pop in the function.
	 * r0 is temporary and r1 is zero.
	 */
	register const uint8_t count_delay asm("r18") = 59; // 59 * 4, 236 Clocks
	register uint8_t count asm("r19");
	register uint8_t set_bit_pin asm("r20") = _BV( portb_pin_number_for_tx );
	register uint8_t clear_bit_pin asm("r21") = ~( set_bit_pin );;
	register uint8_t i asm("r22") = 9;
	asm volatile (
			"clc" "\n\t" // Clear Carry Bit for Start Bit
		"software_uart_tx_38400_forloop:" "\n\t" // 14 Clocks + Delay 236 Clocks
			"in __tmp_reg__, %[portb]" "\n\t"
			"brcc software_uart_tx_38400_forloop_carryclear" "\n\t" // From Result of lsr
			"or __tmp_reg__, %[set_bit_pin]" "\n\t"
			"rjmp software_uart_tx_38400_forloop_common" "\n\t"
			"software_uart_tx_38400_forloop_carryclear:" "\n\t"
				"and __tmp_reg__, %[clear_bit_pin]" "\n\t"
				"nop" "\n\t"
			"software_uart_tx_38400_forloop_common:" "\n\t"
				"out %[portb], __tmp_reg__" "\n\t"
				// count_delay * 4 Cycles
				"mov %[count], %[count_delay]" "\n\t"
				"software_uart_tx_38400_forloop_common_delay:" "\n\t"
					"nop" "\n\t"
					"subi %[count], 0x1" "\n\t"
					"brne software_uart_tx_38400_forloop_common_delay" "\n\t"
					"rjmp .+0" "\n\t" // Two Cycles
				"subi %[i], 0x1" "\n\t"
				"breq software_uart_tx_38400_stopbit" "\n\t" // Branch If Lower
				"nop" "\n\t"
				"lsr %[character]" "\n\t"
				"rjmp software_uart_tx_38400_forloop" "\n\t"
		"software_uart_tx_38400_stopbit:" "\n\t"
			"in __tmp_reg__, %[portb]" "\n\t"
			"or __tmp_reg__, %[set_bit_pin]" "\n\t"
			"rjmp .+0" "\n\t" // Two Cycles
			"rjmp .+0" "\n\t" // Two Cycles
			"rjmp .+0" "\n\t" // Two Cycles
			"out %[portb], __tmp_reg__" "\n\t"
			//count_delay * 4 Cycles
			"mov %[count], %[count_delay]" "\n\t"
			"inc %[count]" "\n\t" // 60 * 4, 240 Clocks + 10 Clock from out to ret at Last
			"software_uart_tx_38400_stopbit_delay:" "\n\t"
				"nop" "\n\t"
				"subi %[count], 0x1" "\n\t"
				"brne software_uart_tx_38400_stopbit_delay" "\n\t"
				"rjmp .+0" "\n\t" // Two Cycles
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
