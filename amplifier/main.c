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
 * Amplified (Approx. 3 Times) Output from PB0 (OC0A)
 * Input from PB2 (ADC1)
 */

#define SAMPLE_RATE (double)(F_CPU / 256) // 37500 Samples per Seconds

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

volatile uint8_t wave_sync; // Sync with Process in ISR

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	uint8_t value_adc_channel_1_high = 0; // Bit[7:0] Is ADC[9:2]

	/* Initialize Global Variables */
	wave_sync = 0;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */
	PORTB = 0; // All Low
	DDRB = _BV(DDB0); // Bit Value Set PB0 (OC0A) as Output

	/* ADC */
	// For Noise Reduction of ADC, Disable All Digital Input Buffers
	DIDR0 = _BV(ADC0D)|_BV(ADC2D)|_BV(ADC3D)|_BV(ADC1D)|_BV(AIN1D)|_BV(AIN0D);
	// Set ADC, Internal Voltage Reference (1.1V), ADLAR, ADC1 (PB2)
	ADMUX = _BV(REFS0)|_BV(ADLAR)|_BV(MUX0);
	// ADC Auto Trigger Free Running Mode
	ADCSRB = 0;
	// ADC Enable, Start, Set Auto Trigger Enable, Prescaler 16 to Have ADC Clock 600Khz
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2);

	/* Counter/Timer */
	// Counter Reset
	TCNT0 = 0;
	// Clear Output Compare A
	OCR0A = 0;
	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);
	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted and OC0B Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0B1)|_BV(COM0A1);
	// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
	TCCR0B = _BV(CS00);

	// Start to Issue Interrupt
	sei();

	while(1) {
		if ( wave_sync ) {
			value_adc_channel_1_high = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
			OCR0A = value_adc_channel_1_high;
			wave_sync = 0;
		}
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	wave_sync = 1;
}
