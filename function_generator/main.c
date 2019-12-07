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
#include <util/delay.h>
#include <util/delay_basic.h>

/**
 * Output Sawtooth Wave from PB0 (OC0A)
 * Output Square Wave from PB1 (OC0B)
 * Input from PB4 (ADC2) to Determine Output Frequency
 * In default, the voltage vias is set as 0x80, the lowest peak is set as 0x01, the highest peak is set as 0xFF.
 */

#define SAMPLE_RATE (F_CPU / 256) // 37500
uint8_t const voltage_bias = 0x80;
uint8_t const absolute_peak = 0x7F; // For Square Wave

uint16_t sample_count = 0; // 0-255
uint16_t count_per_pi = 0; // Count per Pi (Approx. 3.14) Radian
uint16_t count_per_2pi = 0; // Count per 2Pi Radian

/**
 *                      SAMPLE_RATE
 * count_per_2pi + 1 = -------------
 *                       Frequency
 */

uint16_t delta_sawtooth = 0; // Fixed Point Arithmetic, Bit[15:12] Reserved for Calculation, Bit[11:4] UINT8, Bit[3:0] Fractional Part

/**
 *                    254 (Peak to Peak)
 * delta_sawtooth = ----------------------
 *                      count_per_2pi
 */

uint8_t function_start = 0;

int main(void) {
	uint8_t const select_adc_channel_2 = _BV(MUX1); // ADC2 (PB4)
	uint8_t const clear_adc_channel = ~(_BV(MUX1)|_BV(MUX0));
	uint8_t const start_adc = _BV(ADSC);
	uint8_t value_adc_channel_2_high_buffer = 0; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_2_high = 0; // Bit[7:0] Is ADC[9:2]
	uint16_t count_per_2pi_buffer = 0;

	/* Clock Calibration */

	OSCCAL += 0x03; // Frequency Calibration for Individual Difference at VCC = 3.3V

	/* I/O Settings */

	PORTB = 0; // All Low
	DDRB = 0; // All Input
	DDRB |= _BV(DDB1)|_BV(DDB0); // Bit Value Set PB0 (OC0A) and PB1 (OC0B) as Output

	/* ADC */

	// For Noise Reduction of ADC, Disable All Digital Input Buffers
	DIDR0 = _BV(ADC0D)|_BV(ADC2D)|_BV(ADC3D)|_BV(ADC1D)|_BV(AIN1D)|_BV(AIN0D);

	// Set ADC, Vcc as Reference, ADLAR
	ADMUX = _BV(ADLAR);

	// ADC Enable, Prescaler 64 to Have ADC Clock 150Khz
	ADCSRA = _BV(ADEN)|_BV(ADPS2)|_BV(ADPS1);

	/* Counter/Timer */

	// Counter Reset
	TCNT0 = 0;

	// Set Output Compare A
	OCR0A = voltage_bias;

	// Set Output Compare B
	OCR0B = voltage_bias;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted and OC0B Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0B1)|_BV(COM0A1);

	// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
	TCCR0B = _BV(CS00);

	while(1) {
		ADMUX |= select_adc_channel_2;
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		ADMUX &= clear_adc_channel;

		value_adc_channel_2_high_buffer = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
		if ( value_adc_channel_2_high_buffer != value_adc_channel_2_high ) {
			value_adc_channel_2_high = value_adc_channel_2_high_buffer;
			if ( value_adc_channel_2_high >= 224 ) {
				count_per_2pi_buffer = 39; // Count per 2Pi Radian, 937.5 Hz
			} else if ( value_adc_channel_2_high >= 192 ) {
				count_per_2pi_buffer = 59; // Count per 2Pi Radian, 625 Hz
			} else if ( value_adc_channel_2_high >= 160 ) {
				count_per_2pi_buffer = 79; // Count per 2Pi Radian, 468.75 Hz
			} else if ( value_adc_channel_2_high >= 128 ) {
				count_per_2pi_buffer = 99; // Count per 2Pi Radian, 375 Hz
			} else if ( value_adc_channel_2_high >= 96 ) {
				count_per_2pi_buffer = 119; // Count per 2Pi Radian, Approx. 312.5 Hz
			} else if ( value_adc_channel_2_high >= 64 ) {
				count_per_2pi_buffer = 139; // Count per 2Pi Radian, Approx. 267.86 Hz
			} else if ( value_adc_channel_2_high >= 32 ) {
				count_per_2pi_buffer = 159; // Count per 2Pi Radian, 234.375 Hz
			} else { // ADC Value < 32
				count_per_2pi_buffer = 0;
			}
			if ( count_per_2pi_buffer != count_per_2pi ) {
				if ( count_per_2pi_buffer ) {
					cli(); // Stop to Issue Interrupt
					sample_count = 0;
					count_per_2pi = count_per_2pi_buffer;
					count_per_pi = count_per_2pi >> 1;
					delta_sawtooth = (254 << 8) / (count_per_2pi << 4); // Fixed Point Arithmetic (LSL4 to Dividend)
					function_start = 1;
					sei(); // Start to Issue Interrupt
				} else {
					cli(); // Stop to Issue Interrupt
					sample_count = 0;
					count_per_2pi = 0;
					count_per_pi = 0;
					delta_sawtooth = 0; // Fixed Point Arithmetic (LSL4 to Dividend)
					function_start = 0;
					sei(); // Start to Issue Interrupt
				}
			}
		}
		_delay_ms(2);
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	uint16_t temp1;
	uint16_t temp2;

	if ( function_start ) { // Start Function
		// Saw Tooth Wave
		if ( sample_count < count_per_2pi ) {
			// Prepare for Fixed Point Arithmetic, Bit[11:4] UINT8, Bit[3:0] Fractional Part
			temp1 = sample_count << 4;
			// Fixed Point Arithmetic, Logical Shift Right 8 Times to Make Bit[7:0] UINT8
			temp2 = (delta_sawtooth * temp1) >> 8;
			temp2 += 1; // Lowest Peak 0x01
			if ( temp2 > 0xFF ) temp2 = 0xFF; // Saturate at 8-bit
			OCR0A = temp2;
		} else {
			OCR0A = voltage_bias + absolute_peak;
		}
		// Square Wave
		if ( sample_count <= count_per_pi ) {
			OCR0B = voltage_bias + absolute_peak;
		} else {
			OCR0B = voltage_bias - absolute_peak;
		}
		sample_count++;
		if ( sample_count > count_per_2pi ) sample_count = 0;
	} else { // Stop Function
		OCR0A = voltage_bias;
		OCR0B = voltage_bias;
	}
}
