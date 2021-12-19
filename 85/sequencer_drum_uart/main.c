/**
 * Copyright 2021 Kenta Ishii
 * License: 3-Clause BSD License
 * SPDX Short Identifier: BSD-3-Clause
 */

#define F_CPU 16000000UL // PLL 16.0Mhz to ATtiny85
#include <stdlib.h>
#include <avr/io.h>
#include <avr/cpufunc.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include "sequencer.h"
#include "include/random.h"
#include "include_85/software_uart.h"

#ifndef CALIB_OSCCAL
#define CALIB_OSCCAL 0x00
#warning "CALIB_OSCCAL is defined with the default value 0x00."
#endif

/**
 * PWM Output (OC0A): PB0 (DC Biased)
 * Reserved to Control Transceiver or Device: PB1 (Output with Low)
 * Reserved to Control Transceiver or Device: PB2 (Output with Low)
 * Software UART Tx: PB3
 * Software UART Rx: PB4 (Pulled Up)
 *  0x58 (X): Start and Clock Sequence (1)
 *  0x59 (Y): Start and Clock Sequence (2)
 *  0x50 (P): Stop and Reset Sequence
 *  Note: The set of Bit[3] starts a sequence, and the clear of Bit[3] stops a sequence. Bit[2:0] selects a sequence. Bit[7:4] indentifies a device group (in 4 groups).
 */

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t volume_mask = 0x00;
	uint8_t volume_offset = SEQUENCER_VOLTAGE_BIAS;
	uint8_t random_high_resolution = 0;
	uint16_t count_last = 0;
	uint8_t program_index = 0;
	uint8_t program_byte;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	uint8_t uart_status_buffer_change_last = 0;
	uint8_t uart_byte_last = 0;

	/* Initialize Global Variables */
	random_value = RANDOM_INIT;
	sequencer_count_update = 0;
	sequencer_interval_random = 0;
	sequencer_interval_random_max = 0;
	sequencer_next_random = 0;
	sequencer_is_start = 0;
	software_uart_init();

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* PLL On */
	if ( ! (PLLCSR & _BV(PLLE)) ) PLLCSR |= _BV(PLLE);
	do {
		_delay_us(100);
	} while ( ! (PLLCSR & _BV(PLOCK)) );
	PLLCSR |= _BV(PCKE);

	/* I/O Settings */
	DDRB = _BV(DDB3)|_BV(DDB2)|_BV(DDB1)|_BV(DDB0);
	// To Do: Turn On Transceiver at This Point with Decent Delay
	PORTB = _BV(PB4)|_BV(PB3); // Software UART Rx (PB4) Pullup (There is No Internal Pulldown), and Software UART Tx (PB3) High

	/* Counters */
	// Timer/Counter0: Counter Reset
	TCNT0 = 0;
	// Timer/Counter0: Set Output Compare A
	OCR0A = SEQUENCER_VOLTAGE_BIAS;
	// Timer/Counter1: Counter Reset
	TCNT1 = 0;
	// Timer/Counter1: Set Output Compare A
	OCR1A = 0;
	// Timer/Counter1: Set Output Compare C
	OCR1C = 0xCF; // Decimal 207
	// Set Timer/Counter1 Overflow Interrupt for "ISR(TIMER1_OVF_vect)" and Timer/Counter0 Overflow Interrupt for "ISR(TIMER0_OVF_vect)"
	TIMSK = _BV(TOIE1)|_BV(TOIE0);
	// Timer/Counter0: Select Phase Correct PWM Mode (1) and Output from OC0A Non-inverted
	// Timer/Counter0: Phase Correct Mode (5) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM00)|_BV(COM0A1);
	// Start Counter with I/O-Clock 16.0MHz / ( 1 * 510 ) = Approx. 31372.55Hz
	TCCR0B = _BV(CS00);
	// Timer/Counter1: Start Counter with PLL Clock (64.0MHz / 32) / 208 (OCR1C + 1) = Approx. 9615.38Hz
	TCCR1 = _BV(PWM1A)|_BV(CS12)|_BV(CS11);
	sei(); // Start to Issue Interrupt

	while(1) {
		if ( uart_status_buffer_change_last != (software_uart_rx_status & SOFTWARE_UART_STATUS_RX_BUFFER_CHANGE_BIT) ) {
			uart_status_buffer_change_last = software_uart_rx_status & SOFTWARE_UART_STATUS_RX_BUFFER_CHANGE_BIT;
			uart_byte_last = software_uart_rx_byte_buffer;
			if ( ((uart_byte_last & SEQUENCER_BYTE_GROUP_START_BIT) == SEQUENCER_BYTE_GROUP_START_BIT) && sequencer_is_start ) sequencer_count_update++;
		}
		if ( ((uart_byte_last & SEQUENCER_BYTE_GROUP_START_BIT) == SEQUENCER_BYTE_GROUP_START_BIT) && ! sequencer_is_start ) {
			random_value = RANDOM_INIT; // Reset Random Value
			sequencer_count_update = 1;
			sequencer_interval_random = 0;
			sequencer_interval_random_max = 0;
			count_last = 0;
			sequencer_is_start = 1;
		} else if ( ((uart_byte_last & SEQUENCER_BYTE_GROUP_START_BIT) == SEQUENCER_BYTE_GROUP_BIT) && sequencer_is_start ) {
			sequencer_is_start = 0;
			OCR0A = SEQUENCER_VOLTAGE_BIAS;
			sequencer_next_random = 0;
		}
		if ( sequencer_count_update != count_last ) {
			if ( sequencer_count_update > SEQUENCER_PROGRAM_COUNTUPTO ) { // If Count Reaches Last
				sequencer_count_update = 1;
			}
			count_last = sequencer_count_update;
			program_index = uart_byte_last & SEQUENCER_BYTE_PROGRAM_MASK;
			// Prevent Memory Overflow
			if ( program_index >= SEQUENCER_PROGRAM_LENGTH ) program_index = SEQUENCER_PROGRAM_LENGTH - 1;
			// Prevent Memory Overflow in Case That Doesn't Happen Logically
			//if ( ! count_last ) count_last = 1;
			program_byte = pgm_read_byte(&(sequencer_program_array[program_index][count_last - 1]));
			sequencer_interval_random_max = pgm_read_word(&(sequencer_interval_random_max_array[program_byte & 0xF]));
			volume_mask = pgm_read_byte(&(sequencer_volume_mask_array[(program_byte & 0x70) >> 4]));
			volume_offset = pgm_read_byte(&(sequencer_volume_offset_array[(program_byte & 0x70) >> 4]));
			random_high_resolution = program_byte & 0x80;
		}
		if ( sequencer_next_random ) {
			random_make( random_high_resolution );
			OCR0A = ((uint8_t)(random_high_resolution ? random_value : random_value << 1) & volume_mask) + volume_offset;
			sequencer_next_random = 0;
		}
	}
	return 0;
}

ISR(TIMER0_OVF_vect) {
	if ( sequencer_is_start ) {
		if ( ++sequencer_interval_random >= sequencer_interval_random_max ) {
			sequencer_interval_random = 0;
			sequencer_next_random = 1;
		}
	}
}

ISR(TIMER1_OVF_vect) {
	software_uart_handler_rx_tx( SOFTWARE_UART_HANDLER_RX_TX_MODE_LOOP_BACK );
}
