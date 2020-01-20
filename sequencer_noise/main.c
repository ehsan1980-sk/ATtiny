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
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#define CALIB_OSCCAL 0x03 // Frequency Calibration for Individual Difference at VCC = 3.3V
#define VOLTAGE_BIAS 0x80 // Decimal 128 on Noise Off

#define RANDOM_INIT 0x4000 // Initial Value to Making Random Value, Must Be Non-zero
void random_make( uint8_t bool_high ); // bool_high: True (Not Zero) = 15-bit LFSR-2 (32767 Cycles), Flase (Zero) = 7-bit LFSR-2 (127 Cycles)
uint16_t random_value;

/**
 * Output from PB0 (OC0A)
 * Input from PB1 (Trigger Bit[0]), Set by Detecting Low
 * Input from PB2 (Trigger Bit[1]), Set by Detecting Low
 * Input from PB3 (Trigger Bit[2]), Set by Detecting Low
 * Trigger Bit[2:0]:
 *     0b000: Stop Sequencer
 *     0b001: Play Sequence No.1
 *     0b010: Play Sequence No.2
 *     0b011: PLay Sequence No.3
 *     0b100: PLay Sequence No.4
 *     ...
 * Note that PB4 is reserved as a digital input (pulled-up).
 */

#define SAMPLE_RATE (double)(F_CPU / 256) // 37500 Samples per Seconds
#define SEQUENCER_INTERVAL 375 // Approx. 100Hz = 0.01 Seconds
#define SEQUENCER_COUNTUPTO 64 // 0.01 Seconds * 64
#define SEQUENCER_SEQUENCENUMBER 3 // Maximum Number of Sequence
#define INPUT_SENSITIVITY 250 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint16_t sequencer_interval_count;
uint16_t sequencer_count_update;
uint8_t sequencer_volume;

// Delay Time in Tunrs to Generate Next Random Value
uint16_t const array_type[16] PROGMEM = { // Array in Program Space
	0,
	1,
	2,
	4,
	8,
	16,
	32,
	64,
	128,
	256,
	512,
	1024,
	2048,
	4096,
	8192,
	16384,
};

uint8_t const array_volume_mask[8] PROGMEM = { // Array in Program Space
	0x00,
	0x01, // Up to Decimal 1
	0x03, // Up to Decimal 3
	0x07, // Up to Decimal 7
	0x0F, // Up to Decimal 15
	0x1F, // Up to Decimal 31
	0x3F, // Up to Decimal 63
	0x7F  // Up to Decimal 127
};

uint8_t const array_volume_offset[8] PROGMEM = { // Array in Program Space
	VOLTAGE_BIAS,
	0x7F, // Decimal 127
	0x7E, // Decimal 126
	0x7C, // Decimal 124
	0x78, // Decimal 120
	0x70, // Decimal 112
	0x60, // Decimal 96
	0x40  // Decimal 64
};

/**
 * Bit[3:0]: Index of array_type (0-15)
 * Bit[6:4]: Index of array_volume (0-7)
 * Bit[7]: 0 as 7-bit LFSR-2, 1 as 15-bit LFSR-2
 */
uint8_t const sequencer_array_a[SEQUENCER_SEQUENCENUMBER][SEQUENCER_COUNTUPTO] PROGMEM = { // Array in Program Space
	{0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,0xE0,
	 0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0xD0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,0xC0,
	 0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0xB0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,0xA0,
	 0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x8F}, // Sequence No.1
	{0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,
	 0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,
	 0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,0x70,
	 0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xF0,0xFF}, // Sequence No.2
	{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}  // Sequence No.3
};


int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_input = _BV(PINB3)|_BV(PINB2)|_BV(PINB1); // Assign PB3, PB2 and PB1 as Trigger Bit[1:0]
	uint8_t const pin_input_shift = PINB1;
	uint16_t count_delay;
	uint16_t max_count_delay;
	uint8_t volume_mask;
	uint8_t volume_offset;
	uint8_t noise_cycle;
	uint8_t start_noise;
	uint16_t sequencer_count_last = 0;
	uint8_t input_pin;
	uint8_t input_pin_last = 0;
	uint8_t input_pin_buffer = 0;
	uint8_t input_pin_buffer_last = 0;
	uint8_t sequencer_array_a_index = 0;
	uint8_t sequencer_byte;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	uint16_t input_sensitivity_count = INPUT_SENSITIVITY;

	/* Initialize Global Variables */
	sequencer_interval_count = 0;
	sequencer_count_update = 0;
	sequencer_volume = VOLTAGE_BIAS;
	random_value = RANDOM_INIT;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */
	DDRB = _BV(DDB0);
	PORTB = _BV(PB4)|_BV(PB3)|_BV(PB2)|_BV(PB1); // Pullup Button Input (There is No Internal Pulldown)

	/* Counter/Timer */
	// Counter Reset
	TCNT0 = 0;
	// Set Output Compare A
	OCR0A = VOLTAGE_BIAS;
	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);
	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0A1);
	// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
	TCCR0B = _BV(CS00);
	// Initialize Local Variables Before Loop
	count_delay = 1;
	max_count_delay = 0;
	volume_mask = 0x00;
	volume_offset = VOLTAGE_BIAS;
	noise_cycle = 1;
	start_noise = 0;

	while(1) {
		input_pin = ((PINB ^ pin_input) & pin_input) >> pin_input_shift;
		if ( input_pin >= SEQUENCER_SEQUENCENUMBER ) input_pin = SEQUENCER_SEQUENCENUMBER;
		if ( input_pin == input_pin_last ) { // If Match
			if ( ! --input_sensitivity_count ) { // If Count Reaches Zero
				input_pin_buffer = input_pin;
				input_sensitivity_count = INPUT_SENSITIVITY;
			}
		} else { // If Not Match
			input_pin_last = input_pin;
			input_sensitivity_count = INPUT_SENSITIVITY;
		}
		if ( input_pin_buffer != input_pin_buffer_last ) {
			input_pin_buffer_last = input_pin_buffer;
			if ( input_pin_buffer_last ) { // If Not Zero
				sequencer_array_a_index = input_pin_buffer_last - 1;
				sequencer_interval_count = 0;
				sequencer_count_update = 0;
				sequencer_count_last = 0;
				TCNT0 = 0; // Counter Reset
				TIFR0 |= _BV(TOV0); // Clear Set Timer/Counter0 Overflow Flag by Logic One
				if ( ! start_noise ) start_noise = 1;
				if ( ! (SREG & _BV(SREG_I)) ) sei(); // If Global Interrupt Enable Flag Is Not Set, Start to Issue Interrupt
			}
		}
		if ( sequencer_count_update != sequencer_count_last ) {
			if ( sequencer_count_update > SEQUENCER_COUNTUPTO ) { // If Count Reaches Last
				max_count_delay = 0;
				volume_mask = 0x00;
				volume_offset = VOLTAGE_BIAS;
				noise_cycle = 1;
				start_noise = 0;
				// Reset Random Value
				random_value = RANDOM_INIT;
				sequencer_count_update = SEQUENCER_COUNTUPTO + 1;
				cli(); // Stop to Issue Interrupt
			}
			sequencer_count_last = sequencer_count_update;
			if ( sequencer_count_last <= SEQUENCER_COUNTUPTO ) {
				sequencer_byte = pgm_read_byte(&(sequencer_array_a[sequencer_array_a_index][sequencer_count_last - 1]));
				max_count_delay = pgm_read_byte(&(array_type[sequencer_byte & 0xF]));
				volume_mask = pgm_read_byte(&(array_volume_mask[(sequencer_byte & 0x70) >> 4]));
				volume_offset = pgm_read_byte(&(array_volume_offset[(sequencer_byte & 0x70) >> 4]));
				noise_cycle = sequencer_byte & 0x80;
			}
		}
		if ( count_delay > max_count_delay ) {
			if ( start_noise ) random_make( noise_cycle );
			count_delay = 0;
			OCR0A = (random_value & volume_mask) + volume_offset;
		}
		count_delay++;
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	sequencer_interval_count++;
	if ( sequencer_interval_count >= SEQUENCER_INTERVAL ) {
		sequencer_interval_count = 0;
		sequencer_count_update++;
	}
}

void random_make( uint8_t bool_high ) {
	random_value = (random_value >> 1)|((((random_value & 0x2) >> 1)^(random_value & 0x1)) << (bool_high ? 14 : 6));
}
