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
 * the error with 3.75 percents phase shift is shared by Rx device and Tx device, so it should be up to 1.875 percents for each device.
 */

/**
 * In my experience, one increment or decrement of OSCCAL changes the frequency of the internal oscillator at 0.5% - 0.65%.
 * Therefore, I set the value of threshold to adjust the oscillator at 0.5% of the frequency of the sample rate.
 */

#define SOFTWARE_UART_PIN_TX PB3 // Use undef to Redefine Pinout
#define SOFTWARE_UART_PIN_RX PINB4 // Use undef to Redefine Pinout
#define SOFTWARE_UART_DATA_BIT_NUMBER 8 // Must Be Maximum 8
#define SOFTWARE_UART_STOP_BIT_NUMBER 1 // Must Be Minimum 1
#define SOFTWARE_UART_BAUD_RATE 1200
#define SOFTWARE_UART_INTERVAL_RX_FIRST 12
#define SOFTWARE_UART_INTERVAL 8
#define SOFTWARE_UART_STATUS_RX_COUNTER_BIT_MASK 0x0F
#define SOFTWARE_UART_STATUS_RX_BUFFER_CHANGE_BIT (0b1 << 4)
#define SOFTWARE_UART_STATUS_RX_FREQ_COUNTER_START_BIT (0b1 << 7)
#define SOFTWARE_UART_FREQUENCY 80 // Hz
#define SOFTWARE_UART_COMPARE_VALUE (SOFTWARE_UART_BAUD_RATE * SOFTWARE_UART_INTERVAL)
#define SOFTWARE_UART_COMPARE_THRESHOLD 48 // 0.5% of SOFTWARE_UART_COMPARE_VALUE

volatile uint8_t software_uart_tx_count;
volatile uint8_t software_uart_tx_interval_count;
volatile uint8_t software_uart_tx_byte;
volatile uint8_t software_uart_rx_status;
volatile uint8_t software_uart_rx_interval_count;
volatile uint8_t software_uart_rx_byte;
volatile uint8_t software_uart_rx_byte_buffer;
volatile uint16_t software_uart_freq_counter_handler_loop;
volatile uint16_t software_uart_freq_counter_byte;

static inline void software_uart_init() {
	software_uart_tx_count = 0;
	software_uart_tx_interval_count = SOFTWARE_UART_INTERVAL;
	software_uart_tx_byte = 0;
	software_uart_rx_status = 0;
	software_uart_rx_interval_count = SOFTWARE_UART_INTERVAL;
	software_uart_rx_byte = 0;
	software_uart_rx_byte_buffer = 0;
	software_uart_freq_counter_handler_loop = 0;
	software_uart_freq_counter_byte = 0;
}

/**
 * handler_rx_tx_mode:
 * Bit[0]: Clear = Normal, 1 = Loop Back
 * Bit[1]: Clear = Only Clear Counter, 1 = 1 = Compare to Adjust OSCCAL and Clear Counters
 */
#define SOFTWARE_UART_HANDLER_RX_TX_MODE_LOOP_BACK_BIT (0b1 << 1)
#define SOFTWARE_UART_HANDLER_RX_TX_MODE_ADJUST_OSC_BIT (0b1 << 2)
static inline void software_uart_handler_rx_tx( uint8_t handler_rx_tx_mode ) {
	uint8_t uart_is_high;
	uint8_t uart_status_rx_counter;
	int16_t compare_counter;
	uart_is_high = (PINB & _BV(SOFTWARE_UART_PIN_RX)) >> SOFTWARE_UART_PIN_RX; // Shift Right to Make 0b1 for Further Process
	uart_status_rx_counter = software_uart_rx_status & SOFTWARE_UART_STATUS_RX_COUNTER_BIT_MASK;
	if ( ! uart_status_rx_counter ) {
		if ( ! uart_is_high ) {
			software_uart_rx_status += 0b1;
			software_uart_rx_interval_count = SOFTWARE_UART_INTERVAL_RX_FIRST;
			software_uart_rx_byte = 0;
			if ( ! (software_uart_rx_status & SOFTWARE_UART_STATUS_RX_FREQ_COUNTER_START_BIT) ) {
				software_uart_freq_counter_handler_loop = 0;
				software_uart_rx_status |= SOFTWARE_UART_STATUS_RX_FREQ_COUNTER_START_BIT;
			}
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
						if ( handler_rx_tx_mode & SOFTWARE_UART_HANDLER_RX_TX_MODE_LOOP_BACK_BIT ) {
							software_uart_tx_byte = software_uart_rx_byte;
							software_uart_tx_count = 9;
						}
						software_uart_freq_counter_byte++;
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
	software_uart_freq_counter_handler_loop++;
	if ( software_uart_freq_counter_byte >= SOFTWARE_UART_FREQUENCY ) {
		compare_counter = software_uart_freq_counter_handler_loop - SOFTWARE_UART_COMPARE_VALUE;
		software_uart_freq_counter_handler_loop = 0;
		software_uart_freq_counter_byte = 0;
		software_uart_rx_status &= ~(SOFTWARE_UART_STATUS_RX_FREQ_COUNTER_START_BIT);
		if ( handler_rx_tx_mode & SOFTWARE_UART_HANDLER_RX_TX_MODE_ADJUST_OSC_BIT ) {
			if ( compare_counter >= SOFTWARE_UART_COMPARE_THRESHOLD ) {
				if ( OSCCAL > 0 ) OSCCAL--;
			} else if ( compare_counter <= -SOFTWARE_UART_COMPARE_THRESHOLD ) {
				if ( OSCCAL < 0x80 ) OSCCAL++;
			}
		}
	}
}
