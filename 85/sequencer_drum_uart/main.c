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

#define CALIB_OSCCAL -0x04 // Frequency Calibration for Individual Difference at VCC = 3.3V

/**
 * PWM Output (OC0A): PB0 (DC Biased)
 * Reserved: PB1 (Pulled Up)
 * Software UART Tx: PB2
 * Button 2: PB3 (Pulled Up), Change Output Level
 * Software UART Rx: PB4 (Pulled Up)
 *  0x58 (X): Start and Clock Sequence (1)
 *  0x59 (Y): Start and Clock Sequence (2)
 *  0x50 (P): Stop and Reset Sequence
 *  Note: The set of Bit[3] starts a sequence, and the clear of Bit[3] stops a sequence. Bit[2:0] selects a sequence. Bit[7:4] indentifies a device group (in 4 groups).
 */

#define RANDOM_INIT 0x4000 // Initial Value to Making Random Value, Must Be Non-zero
inline void random_make( uint8_t high_resolution ); // high_resolution: True (Not Zero) = 15-bit LFSR-2 (32767 Cycles), Flase (Zero) = 7-bit LFSR-2 (127 Cycles)
uint16_t random_value;

#define SEQUENCER_VOLTAGE_BIAS 0x80 // Decimal 128 on Noise Off
#define SEQUENCER_SAMPLE_RATE (double)(F_CPU / 510) // Approx. 31372.55 Samples per Seconds
#define SEQUENCER_INTERVAL_NUMBER 9
#define SEQUENCER_INTERVAL_INDEX_DEFAULT 0
#define SEQUENCER_PROGRAM_COUNTUPTO 64
#define SEQUENCER_PROGRAM_LENGTH 2 // Length of Sequence
#define SEQUENCER_LEVEL_SHIFT_MAX 3
#define SEQUENCER_BUTTON_SENSITIVITY 2500 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)
#define SEQUENCER_BYTE_GROUP_BIT 0x50
#define SEQUENCER_BYTE_START_BIT 0x08
#define SEQUENCER_BYTE_GROUP_START_BIT (SEQUENCER_BYTE_GROUP_BIT|SEQUENCER_BYTE_START_BIT)
#define SEQUENCER_BYTE_PROGRAM_MASK 0x07
#define SOFTWARE_UART_PIN_TX PB2
#define SOFTWARE_UART_PIN_RX PINB4
#define SOFTWARE_UART_DATA_BIT_NUMBER 8 // Must Be Maximum 8
#define SOFTWARE_UART_STOP_BIT_NUMBER 1 // Must Be Minimum 1
#define SOFTWARE_UART_INTERVAL_RX_FIRST 6
#define SOFTWARE_UART_INTERVAL 4
#define SOFTWARE_UART_STATUS_RX_COUNTER_MASK 0x0F
#define SOFTWARE_UART_STATUS_RX_BUFFER_CHANGE_BIT 0x10

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint16_t sequencer_interval_max;
uint16_t sequencer_interval_count;
uint16_t sequencer_count_update;
uint16_t sequencer_interval_random;
uint16_t sequencer_interval_random_max;
uint8_t sequencer_next_random;
uint8_t sequencer_is_start;

// Interval (31250 Divided by Beats in 1 Second)
uint16_t const sequencer_interval_array[SEQUENCER_INTERVAL_NUMBER] PROGMEM = { // Array in Program Space
	3906, // 8 Beats
	3472, // 9 Beats
	3125, // 10 Beats
	2841, // 11 Beats
	2604, // 12 Beats
	2404, // 13 Beats
	2232, // 14 Beats
	2083, // 15 Beats
	1953 // 16 Beats
};

// Delay Time in Turns to Generate Next Random Value
uint16_t const sequencer_interval_random_max_array[16] PROGMEM = { // Array in Program Space
	1,
	2,
	3,
	4,
	8,
	16,
	32,
	64,
	128,
	192,
	256,
	384,
	512,
	768,
	1024,
	1536
};

uint8_t const sequencer_volume_mask_array[8] PROGMEM = { // Array in Program Space
	0x00,
	0x07, // Up to Decimal 7
	0x0F, // Up to Decimal 15
	0x1F, // Up to Decimal 31
	0x3F, // Up to Decimal 63
	0x7F, // Up to Decimal 127
	0xBF, // Up to Decimal 191
	0xFF // Up to Decimal 255
};

uint8_t const sequencer_volume_offset_array[8] PROGMEM = { // Array in Program Space
	SEQUENCER_VOLTAGE_BIAS,
	0x7C, // Decimal 124
	0x78, // Decimal 120
	0x70, // Decimal 112
	0x60, // Decimal 96
	0x40, // Decimal 64
	0x20, // Decimal 32
	0x00 // Decimal 0
};

/**
 * Bit[3:0]: Index of sequencer_interval_random_max_array (0-15)
 * Bit[6:4]: Index of sequencer_volume_mask_array and sequencer_volume_offset_array (0-7)
 * Bit[7]: 0 as 7-bit LFSR-2, 1 as 15-bit LFSR-2
 */
uint8_t const sequencer_program_array[SEQUENCER_PROGRAM_LENGTH][SEQUENCER_PROGRAM_COUNTUPTO] PROGMEM = { // Array in Program Space
	{0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,
	 0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,
	 0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,
	 0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00}, // Sequence Index No. 0
	{0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xF0,0xF5,0xF0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xF0,0xF5,0xF0,0xF5,0xF0,0xF5,0xF0} // Sequence Index No. 1
};

/**
 * To accept a signal error with 50 percents per baud, get a sample at the almost center of a single bit signal in a UART Rx device.
 * If the sample rate is 4 times as much as baud rate, it would have 25 percents phase shift of a single bit signal.
 * If you sent 8 bits, which you need 10 bits because of addition of start and stop bits,
 * the acceptable error of baud rate is that (50 - 25) divided by 10 = 2.5 percents.
 * the error with 2.5 percents phase shift is shared by Rx device and Tx device, so it should be up to 1.25 percents for each device.
 */

uint8_t software_uart_tx_count;
uint8_t software_uart_tx_interval_count;
uint8_t software_uart_tx_byte;
uint8_t software_uart_rx_status;
uint8_t software_uart_rx_interval_count;
uint8_t software_uart_rx_byte;
uint8_t software_uart_rx_byte_buffer;

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_button_2 = _BV(PINB3);
	uint8_t volume_mask = 0x00;
	uint8_t volume_offset = SEQUENCER_VOLTAGE_BIAS;
	uint8_t random_high_resolution = 0;
	uint16_t count_last = 0;
	uint8_t interval_index = SEQUENCER_INTERVAL_INDEX_DEFAULT;
	uint8_t program_index = 0;
	uint8_t program_byte;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	int16_t button_2_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
	uint8_t level_shift = 0;
	uint8_t uart_status_buffer_change_last = 0;
	uint8_t uart_byte_last = 0;

	/* Initialize Global Variables */
	random_value = RANDOM_INIT;
	sequencer_interval_max = pgm_read_word(&(sequencer_interval_array[interval_index]));
	sequencer_interval_count = 0;
	sequencer_count_update = 0;
	sequencer_interval_random = 0;
	sequencer_interval_random_max = 0;
	sequencer_next_random = 0;
	sequencer_is_start = 0;
	software_uart_tx_count = 0;
	software_uart_tx_interval_count = SOFTWARE_UART_INTERVAL;
	software_uart_tx_byte = 0;
	software_uart_rx_status = 0;
	software_uart_rx_interval_count = SOFTWARE_UART_INTERVAL;
	software_uart_rx_byte = 0;
	software_uart_rx_byte_buffer = 0;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */
	DDRB = _BV(DDB2)|_BV(DDB0);
	PORTB = _BV(PB4)|_BV(PB3)|_BV(PB1); // Pullup Button Input (There is No Internal Pulldown)

	/* PLL On */
	if ( ! (PLLCSR & _BV(PLLE)) ) PLLCSR |= _BV(PLLE);
	do {
		_delay_us(100);
	} while ( ! (PLLCSR & _BV(PLOCK)) );
	PLLCSR |= _BV(PCKE);

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
			sequencer_interval_count = 0;
			sequencer_count_update = 1;
			sequencer_interval_random = 0;
			sequencer_interval_random_max = 0;
			count_last = 0;
			sequencer_is_start = 1;
		} else if ( ((uart_byte_last & SEQUENCER_BYTE_GROUP_START_BIT) == SEQUENCER_BYTE_GROUP_BIT) && sequencer_is_start ) {
			sequencer_is_start = 0;
			OCR0A = SEQUENCER_VOLTAGE_BIAS;
			sequencer_next_random = 0;
			sequencer_is_start = 0;
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
			OCR0A = (uint8_t)((((int16_t)(((random_high_resolution ? random_value : random_value << 1) & volume_mask) + volume_offset) - SEQUENCER_VOLTAGE_BIAS) >> level_shift) + SEQUENCER_VOLTAGE_BIAS);
			sequencer_next_random = 0;
		}
		if ( (PINB ^ pin_button_2) & pin_button_2 ) { // If Match
			if ( button_2_sensitivity_count >= 0 ) {
				button_2_sensitivity_count--;
				if ( button_2_sensitivity_count == 0 ) { // If Count Reaches Zero
					if ( ++level_shift > SEQUENCER_LEVEL_SHIFT_MAX ) level_shift = 0;
				} // If Count Reaches -1, Do Nothing
			}
		} else { // If Not Match
			button_2_sensitivity_count = SEQUENCER_BUTTON_SENSITIVITY;
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
	uint8_t uart_is_high;
	uint8_t uart_status_rx_counter;
	uart_is_high = (PINB & _BV(SOFTWARE_UART_PIN_RX)) >> SOFTWARE_UART_PIN_RX; // Shift Right to Make 0b1 for Further Process
	uart_status_rx_counter = software_uart_rx_status & SOFTWARE_UART_STATUS_RX_COUNTER_MASK;
	if ( ! uart_status_rx_counter ) {
		if ( ! uart_is_high ) {
			software_uart_rx_status += 0b1;
			software_uart_rx_interval_count = SOFTWARE_UART_INTERVAL_RX_FIRST;
			software_uart_rx_byte = 0;
		}
	} else {
		if ( --software_uart_rx_interval_count == 0 ) {
			software_uart_rx_interval_count = SOFTWARE_UART_INTERVAL;
			if ( uart_status_rx_counter <= SOFTWARE_UART_DATA_BIT_NUMBER ) {
				software_uart_rx_status += 0b1;
				software_uart_rx_byte |= uart_is_high << (uart_status_rx_counter - 1);
			} else {
				if ( uart_is_high ) {
					software_uart_rx_status += 0b1;
					if ( (uart_status_rx_counter - SOFTWARE_UART_DATA_BIT_NUMBER) >= SOFTWARE_UART_STOP_BIT_NUMBER ) {
						software_uart_rx_byte_buffer = software_uart_rx_byte;
						software_uart_rx_status ^= software_uart_rx_status|SOFTWARE_UART_STATUS_RX_BUFFER_CHANGE_BIT; // Clear Counter and Flip Buffer Change Bit
						software_uart_tx_byte = software_uart_rx_byte;
						software_uart_tx_count = 9;
					}
				}
			}
		}
	}
	if ( --software_uart_tx_interval_count == 0 ) {
		software_uart_tx_interval_count = SOFTWARE_UART_INTERVAL;
		if ( software_uart_tx_count > SOFTWARE_UART_DATA_BIT_NUMBER ) {
			PORTB &= ~(_BV(SOFTWARE_UART_PIN_TX));
			--software_uart_tx_count;
		} else if ( software_uart_tx_count > 0 ) {
			if ( software_uart_tx_byte & _BV(SOFTWARE_UART_DATA_BIT_NUMBER - software_uart_tx_count) ) {
				PORTB |= _BV(SOFTWARE_UART_PIN_TX);
			} else {
				PORTB &= ~(_BV(SOFTWARE_UART_PIN_TX));
			}
			--software_uart_tx_count;
		} else {
			PORTB |= _BV(SOFTWARE_UART_PIN_TX);
			software_uart_tx_interval_count += (SOFTWARE_UART_STOP_BIT_NUMBER - 1) * SOFTWARE_UART_INTERVAL;
		}
	}
}

inline void random_make( uint8_t high_resolution ) { // The inline attribute doesn't make a call, but implants codes.
	random_value = (random_value >> 1)|((((random_value & 0x2) >> 1)^(random_value & 0x1)) << (high_resolution ? 14 : 6));
}
