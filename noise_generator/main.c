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

#define CALIB_OSCCAL 0x03 // Frequency Calibration for Individual Difference at VCC = 3.3V
#define NOISE_DELAY_SLOWEST 32768 // Delay Time to Generate Next Random Value in Microseconds for Noise Type 2
#define RANDOM_INIT 0xFFFF // Initial Value to Making Random Value, Must Be Non-zero

/**
 * Output Noise from PB0 (OC0A)
 * Input from PB1 (Bit[0]), Set by Detecting Low
 * Input from PB2 (Bit[1]), Set by Detecting Low
 * Input from PB3 (Bit[2]), Set by Detecting Low
 * Input from PB4 (Bit[3]), Set by Detecting Low
 * Bit[2:0]:
 *     0b0000: Noise Off and High-Z State
 *     0b0001: Noise Off
 *     ...
 *     0b1111: Noise Type 15 (Fastest)
 */

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_button1 = _BV(PINB1); // Assign PB1 as Button Input
	uint8_t const pin_button2 = _BV(PINB2); // Assign PB2 as Button Input
	uint8_t const pin_button3 = _BV(PINB3); // Assign PB3 as Button Input
	uint8_t const pin_button4 = _BV(PINB4); // Assign PB4 as Button Input
	uint8_t const pwm_output_a_start = _BV(COM0A1); // Non-inverted
	uint8_t const pwm_output_a_stop = (uint8_t)(~(_BV(COM0A1)|_BV(COM0A0)));
	uint8_t const output_low = ~(_BV(PB0)); // PB0 (OC0A) Low
	uint8_t const output_start = _BV(DDB0); // Bit Value Set PB0 (OC0A) as Output
	uint8_t const output_stop = ~(_BV(DDB0)); // Bit Value Clear PB0 (OC0A)
	uint16_t random_value;
	uint16_t count_delay;
	uint16_t max_count_delay;
	uint8_t input_pin;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL

	/* Clock Calibration */

	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */

	DDRB = 0; // All Input
	PORTB = _BV(PB1)|_BV(PB2)|_BV(PB3)|_BV(PB4); // Pullup Button Input (There is No Internal Pulldown)
	//_NOP(); // Wait for Synchronization

	/* Counter/Timer */

	// Counter Reset
	TCNT0 = 0;

	// Select Fast PWM Mode (3)
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00);

	// Initialize Local Variables Before Loop
	random_value = RANDOM_INIT;
	count_delay = 1;

	while(1) {
		input_pin = 0;
		if ( ! (PINB & pin_button1) ) {
			input_pin |= 0b0001;
		}
		if ( ! (PINB & pin_button2) ) {
			input_pin |= 0b0010;
		}
		if ( ! (PINB & pin_button3) ) {
			input_pin |= 0b0100;
		}
		if ( ! (PINB & pin_button4) ) {
			input_pin |= 0b1000;
		}
		max_count_delay = NOISE_DELAY_SLOWEST >> input_pin;
		if ( input_pin ) { // Output Noise
			if ( count_delay > max_count_delay ) {
				srand(random_value - (TCNT0<<8|TCNT0)); // uint16_t
				random_value = rand();
				count_delay = 0;
			}
			count_delay++;
			// Invert Value in input_pin to Make Volume by Logical Shift Right
			OCR0A = random_value;
			// Start Output
			if ( ! ( DDRB & output_start ) ) {
				// Bit Value Set PB0 (OC0A) as Output
				DDRB |= output_start;
				// PWM Output Start
				TCCR0A |= pwm_output_a_start;
				// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
				TCCR0B = _BV(CS00);
			}
		} else { // No Output
			// Stop Output
			if ( DDRB & output_start ) {
				// Stop Counter
				TCCR0B = 0;
				// PWM Output Stop
				OCR0A = 0;
				TCCR0A &= pwm_output_a_stop;
				// PB1 (OC0B) Low
				PORTB &= output_low;
				// Bit Value Clear PB0 (OC0A), High-Z State
				DDRB &= output_stop;
				// Counter Reset
				TCNT0 = 0;
				// Reset Random Value
				random_value = RANDOM_INIT;
			}
		}
	}
	return 0;
}
