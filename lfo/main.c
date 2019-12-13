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
 * Input from PB2 (ADC1) to Determine Output Frequency
 * Input from PB4 (ADC2) to Calibrate Output Frequency, ADC Value 0 Means -16, ADC Value 255 Means +15
 * In default, the voltage vias is set as 0x80, the lowest peak is set as 0x01, the highest peak is set as 0xFF.
 */

#define SAMPLE_RATE (double)(F_CPU / 510 * 64) // Approx. 294.117647 Samples per Seconds
#define VOLTAGE_BIAS 0x80
#define ABSOLUTE_PEAK 0x7F // For Square Wave
#define PEAK_HIGH (VOLTAGE_BIAS + ABSOLUTE_PEAK)
#define PEAK_LOW (VOLTAGE_BIAS - ABSOLUTE_PEAK)
#define PEAK_TO_PEAK (ABSOLUTE_PEAK * 2)

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint8_t osccal_default; // Calibrated Default Value of OSCCAL
int8_t osccal_tuning; // Tuning Value for Variable Tone
int8_t osccal_calibration; // Calibration Value from ADC2
uint16_t sample_count; // 0-255
uint16_t count_per_pi; // Count per Pi (Approx. 3.14) Radian
uint16_t count_per_2pi; // Count per 2Pi Radian

/**
 *                      SAMPLE_RATE
 * count_per_2pi + 1 = -------------
 *                       Frequency
 */

uint16_t fixed_value_sawtooth; // Fixed Point Arithmetic, Bit[15:12] Reserved for Calculation, Bit[11:4] UINT8, Bit[3:0] Fractional Part
uint16_t fixed_delta_sawtooth; // Fixed Point Arithmetic, Bit[15:12] Reserved for Calculation, Bit[11:4] UINT8, Bit[3:0] Fractional Part

/**
 *                         PEAK_TO_PEAK
 * fixed_delta_sawtooth = ---------------
 *                         count_per_2pi
 */

uint8_t function_start;

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
	uint16_t count_per_2pi_buffer = 0;
	uint16_t fixed_delta_sawtooth_buffer = 0;
	int8_t osccal_calibration_buffer = 0;

	/* Initialize Global Variables */

	osccal_default = OSCCAL;
	osccal_tuning = 0;
	osccal_calibration = 0;
	sample_count = 0;
	count_per_pi = 0;
	count_per_2pi = 0;
	fixed_value_sawtooth = 0;
	fixed_delta_sawtooth = 0;
	function_start = 0;

	/* Clock Calibration */

	osccal_default += 0x03; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

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
	OCR0A = VOLTAGE_BIAS;

	// Set Output Compare B
	OCR0B = VOLTAGE_BIAS;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select PWM (Phase Correct) Mode (1) and Output from OC0A Non-inverted and OC0B Non-inverted
	// PWM (Phase Correct) Mode (5) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM00)|_BV(COM0B1)|_BV(COM0A1);

	// Start Counter with I/O-Clock 9.6MHz / ( 510 * 64 ) = Approx. 294.117647Hz
	TCCR0B = _BV(CS00)|_BV(CS01);

	while(1) {
		ADMUX |= select_adc_channel_1;
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		ADMUX &= clear_adc_channel;
		value_adc_channel_1_high_buffer = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
		if ( value_adc_channel_1_high_buffer != value_adc_channel_1_high ) {
			value_adc_channel_1_high = value_adc_channel_1_high_buffer;
			/* Approx. 294.117647 Samples per Seconds */
			if ( value_adc_channel_1_high >= 224 ) {
				count_per_2pi_buffer = 36; // 8 Hz
				osccal_tuning = 1;
			} else if ( value_adc_channel_1_high >= 192 ) {
				count_per_2pi_buffer = 72; // 4 Hz
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 160 ) {
				count_per_2pi_buffer = 146; // 2 Hz
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 128 ) {
				count_per_2pi_buffer = 293; // 1 Hz
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 96 ) {
				count_per_2pi_buffer = 587; // 0.5 Hz
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 64 ) {
				count_per_2pi_buffer = 1175; // 0.25 Hz
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 32 ) {
				count_per_2pi_buffer = 2352; // 0.125 Hz
				osccal_tuning = 0;
			} else { // ADC Value < 32
				count_per_2pi_buffer = 0;
				osccal_tuning = 0;
			}
			if ( count_per_2pi_buffer != count_per_2pi ) {
				if ( count_per_2pi_buffer ) {
					// Fixed Point Arithmetic (DIV), LSL4 to Dividend
					fixed_delta_sawtooth_buffer = ((PEAK_TO_PEAK << 4) << 4) / (count_per_2pi_buffer << 4);
					cli(); // Stop to Issue Interrupt
					sample_count = 0;
					count_per_2pi = count_per_2pi_buffer;
					count_per_pi = count_per_2pi >> 1;
					fixed_delta_sawtooth = fixed_delta_sawtooth_buffer;
					function_start = 1;
					OSCCAL = osccal_default + osccal_tuning + osccal_calibration;
					sei(); // Start to Issue Interrupt
				} else {
					fixed_delta_sawtooth_buffer = 0;
					cli(); // Stop to Issue Interrupt
					sample_count = 0;
					count_per_2pi = 0;
					count_per_pi = 0;
					fixed_delta_sawtooth = 0;
					function_start = 0;
					OSCCAL = osccal_default + osccal_tuning + osccal_calibration;
					sei(); // Start to Issue Interrupt
				}
			}
		}

		ADMUX |= select_adc_channel_2;
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		ADMUX &= clear_adc_channel;
		value_adc_channel_2_high_buffer = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
		if ( value_adc_channel_2_high_buffer != value_adc_channel_2_high ) {
			value_adc_channel_2_high = value_adc_channel_2_high_buffer;
			// Convert (0)-(255) to (-128)-(127) by EOR with 0x80, Arithmetic Logical Shift Right for Range (-16)-(15)
			osccal_calibration_buffer = ((int8_t)(0x80^(value_adc_channel_2_high)) >> 3);
			if ( osccal_calibration_buffer !=  osccal_calibration ) {
				osccal_calibration = osccal_calibration_buffer;
				OSCCAL = osccal_default + osccal_tuning + osccal_calibration;
			}
		}
		_delay_ms(2);
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	uint16_t temp;

	if ( function_start ) { // Start Function
		// Saw Tooth Wave
		if ( sample_count == 0 ) {
			OCR0A = PEAK_LOW;
			fixed_value_sawtooth = PEAK_LOW << 4;
		} else if ( sample_count < count_per_2pi ) {
			fixed_value_sawtooth += fixed_delta_sawtooth; // Fixed Point Arithmetic (ADD)
			temp = fixed_value_sawtooth >> 4; // Make Bit[7:0] UINT8
			if ( 0x0008 & fixed_value_sawtooth ) temp++; // Check Fractional Part Bit[3] (0.5) to Round Off
			if ( temp > 0xFF ) temp = 0xFF; // Saturate at 8-bit
			OCR0A = temp;
		} else {
			OCR0A = PEAK_HIGH;
		}
		// Square Wave
		if ( sample_count <= count_per_pi ) {
			OCR0B = PEAK_HIGH;
		} else {
			OCR0B = PEAK_LOW;
		}
		sample_count++;
		if ( sample_count > count_per_2pi ) sample_count = 0;
	} else { // Stop Function
		OCR0A = VOLTAGE_BIAS;
		OCR0B = VOLTAGE_BIAS;
	}
}
