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
 * Amplified (Approx. 6 Times at 3.3V) Output from PB0 (OC0A)
 * Input from PB2 (ADC1)
 * Input from PB3 Gain (Bit[0]), Set by Detecting Low
 * Input from PB4 Gain (Bit[1]), Set by Detecting Low
 * Gain Bit[1:0]:
 *     0b00: Gain 0dB (Voltage), Multiplier 0
 *     0b01: Gain Approx. 6dB, Multiplier 2
 *     0b10: Gain Approx. 12dB, Multiplier 4
 *     0b11: Gain Approx. 18dB, Multiplier 8
 * Note: The reference voltage of ADC is 1.1V.
 *       For example, at 3.3V for VCC, gain approx. 9.5dB (Multiplier 3) is added to the value determined by Gain Bit[1:0].
 */

#define SAMPLE_RATE (double)(F_CPU / 256) // 37500 Samples per Seconds
#define ADC_BIAS 465 // Bias 0.5V to Internal Reference 1.1V
#define INPUT_SENSITIVITY 250 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)

typedef union _adc16 {
	struct _value8 {
		uint8_t lower; // Bit[7:0] = ADC[7:0]
		uint8_t upper; // Bit[1:0] = ADC[9:8]
	} value8;
	int16_t value16;
} adc16;

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

volatile uint8_t wave_sync; // Sync with Process in ISR

int main(void) {

	/* Declare and Define Local Constants and Variables */
	uint8_t const pin_input = _BV(PINB4)|_BV(PINB3); // Assign PB3, PB2 and PB1 as Trigger Bit[1:0]
	uint8_t const pin_input_shift = PINB3;
	uint16_t input_sensitivity_count = INPUT_SENSITIVITY;
	adc16 adc_channel_1;
	uint8_t input_pin;
	uint8_t input_pin_last = 0;
	uint8_t input_pin_buffer = 0;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL

	/* Initialize Global Variables */
	wave_sync = 0;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */
	PORTB = _BV(PB4)|_BV(PB3); // Pullup Button Input (There is No Internal Pulldown)
	DDRB = _BV(DDB0); // Bit Value Set PB0 (OC0A) as Output

	/* ADC */
	// For Noise Reduction of ADC, Disable All Digital Input Buffers
	DIDR0 = _BV(ADC0D)|_BV(ADC1D)|_BV(AIN1D)|_BV(AIN0D);
	// Set ADC, Internal Voltage Reference (1.1V), ADC1 (PB2)
	ADMUX = _BV(REFS0)|_BV(MUX0);
	// ADC Auto Trigger Free Running Mode
	ADCSRB = 0;
	// ADC Enable, Start, Set Auto Trigger Enable, Prescaler 16 to Have ADC Clock 600Khz
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2);

	/* Counter/Timer */
	// Counter Reset
	TCNT0 = 0;
	// Set Output Compare A
	OCR0A = ADC_BIAS >> 2;
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
			input_pin = ((PINB ^ pin_input) & pin_input) >> pin_input_shift;
			if ( input_pin == input_pin_last ) { // If Match
				if ( ! --input_sensitivity_count ) { // If Count Reaches Zero
					input_pin_buffer = input_pin;
					input_sensitivity_count = INPUT_SENSITIVITY;
				}
			} else { // If Not Match
				input_pin_last = input_pin;
				input_sensitivity_count = INPUT_SENSITIVITY;
			}

			adc_channel_1.value8.lower = ADCL;
			adc_channel_1.value8.upper = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
			adc_channel_1.value16 -= ADC_BIAS;
			// Arithmetic Left Shift (Signed Value in Bit[9:0], Bit[15:10] Same as Bit[9])
			adc_channel_1.value16 <<= input_pin_buffer;
			adc_channel_1.value16 += ADC_BIAS;
			if ( adc_channel_1.value16 > 1023 ) {
				adc_channel_1.value16 = 1023;
			} else if ( adc_channel_1.value16 < 0 ) {
				adc_channel_1.value16 = 0;
			}
			adc_channel_1.value16 >>= 2;
			OCR0A = adc_channel_1.value8.lower;
			wave_sync = 0;
		}
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	wave_sync = 1;
}
