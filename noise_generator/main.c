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
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#define CALIB_OSCCAL 0x03 // Frequency Calibration for Individual Difference at VCC = 3.3V
#define VOLTAGE_BIAS 0x80 // Decimal 128 on Noise Off

#define RANDOM_INIT 0x4000 // Initial Value to Making Random Value, Must Be Non-zero
void random_make(); // 15-bit LFSR = 32767 Cycles
uint16_t random_value;

/**
 * Output Noise from PB0 (OC0A)
 * Input from PB1 (Volume Bit[0]), Set by Detecting Low
 * Input from PB2 (Volume Bit[1]), Set by Detecting Low
 * Input from PB3 (Type Bit[0]), Set by Detecting Low
 * Input from PB4 (Type Bit[1]), Set by Detecting Low
 * Volume Bit[1:0]:
 *     0b00: Noise Off and Reset Random Value
 *     0b01: Noise Volume[1] (Small)
 *     0b10: Noise Volume[2] (Medium)
 *     0b11: Noise Volume[3] (Big)
 * Type Bit[1:0]:
 *     0b00: Noise Type[0]
 *     0b01: Noise Tyee[1]
 *     0b10: Noise Type[2]
 *     0b11: Noise Type[3]
 */

// Delay Time in Tunrs to Generate Next Random Value
uint16_t const array_type[4] PROGMEM = { // Array in Program Space
	0,
	3750,
	7500,
	11250
};

uint8_t const array_volume_mask[4] PROGMEM = { // Array in Program Space
	0,
	0x1F, // Up to Decimal 31
	0x3F, // Up to Decimal 63
	0x7F  // Up to Decimal 127
};

uint8_t const array_volume_offset[4] PROGMEM = { // Array in Program Space
	0,
	0x70, // Decimal 112
	0x60, // Decimal 96
	0x40  // Decimal 64
};

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_button1 = _BV(PINB1); // Assign PB1 as Button Input
	uint8_t const pin_button2 = _BV(PINB2); // Assign PB2 as Button Input
	uint8_t const pin_button3 = _BV(PINB3); // Assign PB3 as Button Input
	uint8_t const pin_button4 = _BV(PINB4); // Assign PB4 as Button Input
	uint16_t count_delay;
	uint16_t max_count_delay;
	uint8_t volume_mask;
	uint8_t volume_offset;
	uint8_t input_volume;
	uint8_t input_type;
	uint8_t start_noise = 0;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL

	/* Initialize Global Variables */
	random_value = RANDOM_INIT;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */
	DDRB = _BV(DDB0);
	PORTB = _BV(PB1)|_BV(PB2)|_BV(PB3)|_BV(PB4); // Pullup Button Input (There is No Internal Pulldown)

	/* Counter/Timer */
	// Counter Reset
	TCNT0 = 0;
	// Set Output Compare A
	OCR0A = VOLTAGE_BIAS;
	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
	TCCR0B = _BV(CS00);
	// Initialize Local Variables Before Loop
	count_delay = 1;

	while(1) {
		input_volume = 0;
		input_type = 0;
		if ( ! (PINB & pin_button1) ) {
			input_volume |= 0b01;
		}
		if ( ! (PINB & pin_button2) ) {
			input_volume |= 0b10;
		}
		if ( ! (PINB & pin_button3) ) {
			input_type |= 0b01;
		}
		if ( ! (PINB & pin_button4) ) {
			input_type |= 0b10;
		}
		max_count_delay = pgm_read_byte(&(array_type[input_type]));
		if ( input_volume ) { // Output Noise
			if ( count_delay > max_count_delay ) {
				random_make();
				count_delay = 0;
				volume_mask = pgm_read_byte(&(array_volume_mask[input_volume]));
				volume_offset = pgm_read_byte(&(array_volume_offset[input_volume]));
				OCR0A = (random_value & volume_mask) + volume_offset;
			}
			count_delay++;
			if ( ! start_noise ) start_noise = 1;
		} else {
			if ( start_noise ) {
				// PWM Output at Bias
				OCR0A = VOLTAGE_BIAS;
				// Counter Reset
				TCNT0 = 0;
				// Reset Random Value
				random_value = RANDOM_INIT;
				start_noise = 0;
			}
		}
	}
	return 0;
}

void random_make() {
	random_value = (random_value >> 1)|((((random_value & 0x10) >> 4)^(((random_value & 0x4) >> 2)^(((random_value & 0x2) >> 1)^(random_value & 0x1)))) << 14);
}
