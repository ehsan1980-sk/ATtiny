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

#define CALIB_OSCCAL 0x03 // Frequency Calibration for Individual Difference at VCC = 3.3V

/**
 * PWM Output 1 from PB0 (OC0A)
 * PWM Output 2 from PB1 (OC0B)
 * Input from PB2 (ADC1) for Dimming PWM Output 1, ADC Value 0 Means Turning Off, ADC Value 255 Means Full Power
 * Input from PB4 (ADC2) for Dimming PWM Output 2, ADC Value 0 Means Turning Off, ADC Value 255 Means Full Power
 */

#define SAMPLE_RATE (double)(F_CPU / 510 * 64) // Approx. 294.117647 Samples per Seconds
#define THRESHOLD 5 // 255 Divided by 5 = 51 Steps

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint8_t osccal_default; // Calibrated Default Value of OSCCAL

int main(void) {

	/* Declare and Define Local Constants and Variables */

	uint8_t const select_adc_channel_1 = _BV(MUX0); // ADC1 (PB2)
	uint8_t const select_adc_channel_2 = _BV(MUX1); // ADC2 (PB4)
	uint8_t const clear_adc_channel = ~(_BV(MUX1)|_BV(MUX0));
	uint8_t const start_adc = _BV(ADSC);
	uint8_t value_adc_channel_1_high_buffer = 0; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_1_high = 0; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_2_high_buffer = 0; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_2_high = 0; // Bit[7:0] Is ADC[9:2]
	uint8_t const pwm_output_a_start = _BV(COM0A1); // Non-inverted
	uint8_t const pwm_output_a_stop = (uint8_t)(~(_BV(COM0A1)|_BV(COM0A0)));
	uint8_t const pwm_output_b_start = _BV(COM0B1); // Non-inverted
	uint8_t const pwm_output_b_stop = (uint8_t)(~(_BV(COM0B1)|_BV(COM0B0)));
	uint8_t const output_clear_1 = ~(_BV(PB0)); // PB0 (OC0A) Low
	uint8_t const output_start_1 = _BV(DDB0); // Bit Value Set PB0 (OC0A) as Output
	uint8_t const output_stop_1 = ~(_BV(DDB0)); // Bit Value Clear PB0(OC0A)
	uint8_t const output_clear_2 = ~(_BV(PB1)); // PB1 (OC0B) Low
	uint8_t const output_start_2 = _BV(DDB1); // Bit Value Set PB1 (OC0B) as Output
	uint8_t const output_stop_2 = ~(_BV(DDB1)); // Bit Value Clear PB1(OC0B)

	/* Initialize Global Variables */

	osccal_default = OSCCAL;

	/* Clock Calibration */

	osccal_default += CALIB_OSCCAL;
	OSCCAL = osccal_default;

	/* I/O Settings */

	PORTB = 0; // All Low
	DDRB = 0; // All Input

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

	// Clear Output Compare A
	OCR0A = 0;

	// Clear Output Compare B
	OCR0B = 0;

	// Select PWM (Phase Correct) Mode (1)
	// PWM (Phase Correct) Mode (5) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM00);

	// Start Counter with I/O-Clock 9.6MHz / ( 510 * 64 ) = Approx. 294.117647Hz
	TCCR0B = _BV(CS00)|_BV(CS01);

	while(1) {
		ADMUX |= select_adc_channel_1;
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		ADMUX &= clear_adc_channel;
		value_adc_channel_1_high_buffer = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
		if ( abs( (int8_t)(value_adc_channel_1_high_buffer - value_adc_channel_1_high) ) >= THRESHOLD ) {
			value_adc_channel_1_high = value_adc_channel_1_high_buffer;
			if ( value_adc_channel_1_high ) { // PWM Output 1
				// Start Output
				if ( ! ( DDRB & output_start_1 ) ) {
					// PWM Output 1 Start
					TCCR0A |= pwm_output_a_start;
					// Bit Value Set PB0 (OC0A) as Output
					DDRB |= output_start_1;
					// Set Output Compare A
					OCR0A = value_adc_channel_1_high;
				} else {
					// Set Output Compare A
					OCR0A = value_adc_channel_1_high;
				}
			} else { // No Output
				// Stop Output
				if ( DDRB & output_start_1 ) {
					// PWM Output 1 Stop
					TCCR0A &= pwm_output_a_stop;
					// PB0 (OC0A) Low
					PORTB &= output_clear_1;
					// Bit Value Clear PB0 (OC0A), High-Z State
					DDRB &= output_stop_1;
					// Clear Output Compare A
					OCR0A = 0;
				}
			}
		}

		ADMUX |= select_adc_channel_2;
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		ADMUX &= clear_adc_channel;
		value_adc_channel_2_high_buffer = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
		if ( abs( (int8_t)(value_adc_channel_2_high_buffer - value_adc_channel_2_high) ) >= THRESHOLD ) {
			value_adc_channel_2_high = value_adc_channel_2_high_buffer;

			if ( value_adc_channel_2_high ) { // PWM Output 2
				// Start Output
				if ( ! ( DDRB & output_start_2 ) ) {
					// PWM Output 2 Start
					TCCR0A |= pwm_output_b_start;
					// Bit Value Set PB1 (OC0B) as Output
					DDRB |= output_start_2;
					// Set Output Compare B
					OCR0B = value_adc_channel_2_high;
				} else {
					// Set Output Compare B
					OCR0B = value_adc_channel_2_high;
				}
			} else { // No Output
				// Stop Output
				if ( DDRB & output_start_2 ) {
					// PWM Output 2 Stop
					TCCR0A &= pwm_output_b_stop;
					// PB1 (OC0B) Low
					PORTB &= output_clear_2;
					// Bit Value Clear PB1 (OC0B), High-Z State
					DDRB &= output_stop_2;
					// Clear Output Compare B
					OCR0B = 0;
				}
			}
		}

		_delay_ms(2);
	}
	return 0;
}
