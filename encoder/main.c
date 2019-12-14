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
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/delay_basic.h>

#define CALIB_OSCCAL 0x03 // Frequency Calibration for Individual Difference at VCC = 3.3V

/**
 * PB3 as Output Bit[0] of Encoder
 * PB4 as Output Bit[1] of Encoder
 * PB1 as Output Bit[2] of Encoder
 * PB0 as Output Bit[3] of Encoder
 * Input from PB2 (ADC1): Bit[3:0] of the encoder is calculated through logical shift left 6 times to ADC Value (10-bit).
 */

#define SAMPLE_RATE (double)(F_CPU / 510 * 64) // Approx. 294.117647 Samples per Seconds

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint8_t osccal_default; // Calibrated Default Value of OSCCAL

int main(void) {

	/* Declare and Define Local Constants and Variables */

	uint8_t const select_adc_channel_1 = _BV(MUX0); // ADC1 (PB2)
	uint8_t const clear_adc_channel = ~(_BV(MUX1)|_BV(MUX0));
	uint8_t value_adc_channel_1_high = 0; // Bit[7:0] Is ADC[9:2]
	uint8_t encoder_output;

	/* Initialize Global Variables */

	osccal_default = OSCCAL;

	/* Clock Calibration */

	osccal_default += CALIB_OSCCAL;
	OSCCAL = osccal_default;

	/* I/O Settings */

	PORTB = 0; // All Low
	DDRB = 0; // All Input
	DDRB |= _BV(DDB0)|_BV(DDB1)|_BV(DDB3)|_BV(DDB4);

	/* ADC */

	// For Noise Reduction of ADC, Disable All Digital Input Buffers
	DIDR0 = _BV(ADC0D)|_BV(ADC2D)|_BV(ADC3D)|_BV(ADC1D)|_BV(AIN1D)|_BV(AIN0D);

	// Set ADC, Vcc as Reference, ADLAR
	ADMUX = _BV(ADLAR);

	// ADC Enable, ADC Interrupt Enable, Prescaler 64 to Have ADC Clock 150Khz
	// Memo: Set more speed for ADC Clock (600Khz), although it affects the absolute resolution. Set ADLAR and get only ADCH for 8 bit resolution.
	ADCSRA = _BV(ADEN)|_BV(ADIE)|_BV(ADPS2)|_BV(ADPS1);

	// Set Sleep Mode as ADC Noise Reduction Mode
	set_sleep_mode(SLEEP_MODE_ADC);

	while(1) {
		ADMUX |= select_adc_channel_1;
		sleep_enable();
		sei(); // Start to Issue Interrupt
		sleep_cpu();
		sleep_disable();
		cli(); // Stop to Issue Interrupt
		ADMUX &= clear_adc_channel;
		value_adc_channel_1_high = ADCH >> 4; // ADC[9:0] Will Be Updated After High Bits Are Read
		encoder_output = PORTB;
		if ( value_adc_channel_1_high & 0b0001 ) {
			encoder_output |= _BV(PB3);
		} else {
			encoder_output &= ~(_BV(PB3));
		}
		if ( value_adc_channel_1_high & 0b0010 ) {
			encoder_output |= _BV(PB4);
		} else {
			encoder_output &= ~(_BV(PB4));
		}
		if ( value_adc_channel_1_high & 0b0100 ) {
			encoder_output |= _BV(PB1);
		} else {
			encoder_output &= ~(_BV(PB1));
		}
		if ( value_adc_channel_1_high & 0b1000 ) {
			encoder_output |= _BV(PB0);
		} else {
			encoder_output &= ~(_BV(PB0));
		}
		PORTB = encoder_output;
		_delay_ms(20);
	}
	return 0;
}

// For ADC Noise Reduction Mode
EMPTY_INTERRUPT(ADC_vect);
