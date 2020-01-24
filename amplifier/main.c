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
 * Output from PB0 (OC0A)
 * Input from PB1 Gain (Bit[0]), Set by Detecting Low
 * Input from PB2 Gain (Bit[1]), Set by Detecting Low
 * Gain Bit[1:0]:
 *     0b00: Gain 0dB (Voltage), Multiplier 0
 *     0b01: Gain Approx. 6dB, Multiplier 2
 *     0b10: Gain Approx. 12dB, Multiplier 4
 *     0b11: Gain Approx. 18dB, Multiplier 8
 * Input from PB4 (ADC2)
 * Note: The reference voltage of ADC is 1.1V.
 *       For example, at 3.3V for VCC, gain approx. 9.5dB (Multiplier 3) is added to the value determined by Gain Bit[1:0].
 */

#define SAMPLE_RATE (double)(F_CPU / 256) // 37500 Samples per Seconds
#define INPUT_SENSITIVITY 250 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)
#define BIAS_DETECTION_TURNS 100 // Get Moving Average of ADC Values at Start to Know Voltage Bias
#define CLIP_THRESHOLD 120 // Clip ADC Value over/under Voltage Bias +- This Value

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
	uint8_t const pin_input = _BV(PINB2)|_BV(PINB1); // Assign PB2 and PB1 as Gain Bit[1:0]
	uint8_t const pin_input_shift = PINB1;
	uint16_t input_sensitivity_count = INPUT_SENSITIVITY;
	adc16 adc_sample;
	adc16 adc_bias;
	adc16 adc_clip_upper;
	adc16 adc_clip_under;
	uint8_t input_pin;
	uint8_t input_pin_last = 0;
	uint8_t input_pin_buffer = 0;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	uint8_t value_round;

	/* Initialize Global Variables */
	wave_sync = 0;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */
	PORTB = _BV(PB2)|_BV(PB1); // Pullup Button Input (There is No Internal Pulldown)
	DDRB = _BV(DDB0); // Bit Value Set PB0 (OC0A) as Output

	/* ADC */
	// For Noise Reduction of ADC, Disable Digital Input Buffers
	DIDR0 = _BV(ADC0D)|_BV(ADC3D)|_BV(ADC2D)|_BV(AIN0D);
	// Set ADC, Internal Voltage Reference (1.1V), ADC2 (PB4)
	ADMUX = _BV(REFS0)|_BV(MUX1);
	// ADC Auto Trigger Free Running Mode
	ADCSRB = 0;
	// ADC Enable, Start, Set Auto Trigger Enable, Prescaler 16 to Have ADC Clock 600Khz
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2);

	/* Voltage Bias Detection */
	// Wait for First ADC Value (25 ADC Clocks) and Transient Response Since Power On or Reset
	_delay_ms( 10 );
	adc_bias.value8.lower = ADCL;
	adc_bias.value8.upper = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
	for ( uint16_t i = 0; i < BIAS_DETECTION_TURNS; i++ ) {
		_delay_us( 30 ); // Wait for 13.5 ADC Clocks for Next ADC Value
		adc_sample.value8.lower = ADCL;
		adc_sample.value8.upper = ADCH;
		adc_bias.value16 = (adc_bias.value16 + adc_sample.value16) >> 1;
	}
	adc_clip_upper.value16 = adc_bias.value16 + CLIP_THRESHOLD;
	adc_clip_under.value16 = adc_bias.value16 - CLIP_THRESHOLD;

	/* Counter/Timer */
	// Counter Reset
	TCNT0 = 0;
	// Set Output Compare A
	OCR0A = (uint8_t)(adc_bias.value16 >> 2);
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

			adc_sample.value8.lower = ADCL;
			adc_sample.value8.upper = ADCH;
			adc_sample.value16 -= adc_bias.value16;
			// Arithmetic Left Shift (Signed Value in Bit[9:0], Bit[15:10] Same as Bit[9])
			adc_sample.value16 <<= input_pin_buffer;
			adc_sample.value16 += adc_bias.value16;
			if ( adc_sample.value16 > adc_clip_upper.value16 ) {
				adc_sample.value16 = adc_clip_upper.value16;
			} else if ( adc_sample.value16 < adc_clip_under.value16 ) {
				adc_sample.value16 = adc_clip_under.value16;
			}
			if ( adc_sample.value8.lower & 0b10 ) { // ADC Bit[1] to Be Rounded
				value_round = 1;
			} else {
				value_round = 0;
			}
			adc_sample.value16 >>= 2;
			OCR0A = adc_sample.value8.lower + value_round;
			wave_sync = 0;
		}
	}
	return 0;
}

ISR(TIM0_OVF_vect) {
	wave_sync = 1;
}
