/**
 * Copyright 2021 Kenta Ishii
 * License: 3-Clause BSD License
 * SPDX Short Identifier: BSD-3-Clause
 */

/**
 * To accept a signal error with 50 percents per baud, get a sample at the almost center of a single bit signal in a UART Rx device.
 * If the sample rate is 8 times as much as baud rate, it would have 12.5 percents phase shift of a single bit signal.
 * If you sent 8 bits, which you need 10 bits because of addition of start and stop bits,
 * the acceptable error of baud rate is that (50 - 12.5) divided by 10 = 3.75 percents.
 * the error with 3.75 percents phase shift is shared by Rx device and Tx device, so it should be up to 1.25 percents for each device.
 */

#define SOFTWARE_UART_PIN_TX PB3 // Use undef to Redefine Pinout
#define SOFTWARE_UART_PIN_RX PINB4 // Use undef to Redefine Pinout
#define SOFTWARE_UART_DATA_BIT_NUMBER 8 // Must Be Maximum 8
#define SOFTWARE_UART_STOP_BIT_NUMBER 1 // Must Be Minimum 1
#define SOFTWARE_UART_INTERVAL_RX_FIRST 12
#define SOFTWARE_UART_INTERVAL 8
#define SOFTWARE_UART_STATUS_RX_COUNTER_MASK 0x0F
#define SOFTWARE_UART_STATUS_RX_BUFFER_CHANGE_BIT 0x10

volatile uint8_t software_uart_tx_count;
volatile uint8_t software_uart_tx_interval_count;
volatile uint8_t software_uart_tx_byte;
volatile uint8_t software_uart_rx_status;
volatile uint8_t software_uart_rx_interval_count;
volatile uint8_t software_uart_rx_byte;
volatile uint8_t software_uart_rx_byte_buffer;

static inline void software_uart_handler_rx_tx ( uint8_t handler_mode ) { // handler_mode: 0 = Normal, 1 = Loop Back
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
						if ( handler_mode == 1 ) {
							software_uart_tx_byte = software_uart_rx_byte;
							software_uart_tx_count = 9;
						}
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
