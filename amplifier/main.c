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
 * Note1: The reference voltage of ADC is VCC.
 *        Gain approx. 12dB (Multiplier 4) is added to the value determined by Gain Bit[1:0].
 * Note2: ADC clock rate is set at 600K Hz to have more speed than sampling rate.
 *        One sampling takes (1 / 600K) * 13 = 22 microseconds in ADC.
 *        Whereas, the sampling rate is 37500 samples per second, which takes 1 / 37500 = 27 microseconds.
 * Note3: The latency in the chip is calculated as follows.
 *          27 microseconds (1 / Sampling rate): The sample is obtained from the last polling of the timer interrupt.
 *        + Process Time in Loop: This can be included in the next.
 *        + 27 microseconds (1 / Sampling rate): Maximum Time Needed to Update Compare Value (OCR0A) in PWM
 *        + 27 microseconds (1 / Sampling rate): Generation of Updated PWM Pulse
 *        = 81 microseconds
 */

#define SAMPLE_RATE (double)(F_CPU / 256) // 37500 Samples per Second
#define INPUT_SENSITIVITY 250 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)
#define ADC_BIAS_DEFAULT 512 // 10-bit Unsigned
#define ADC_BIAS_CORRECTION -12 // 10-bit Unsigned, Correction of DC Bias at ADC
#define PWM_BIAS 128 // 8-bit Unsigned
#define PWM_CLIP_UPPER 228 // Clip PWM Value over This Value (8-bit Unsigned), Consider of ADC_BIAS_CORRECTION (240 + Correction)
#define PWM_CLIP_UNDER 4 // Clip PWM Value under This Value (8-bit Unsigned), Consider of ADC_BIAS_CORRECTION (16 + Correction)
#define BIAS_DETECTION_DELAY 500 // Wait Time for End of Transient Response in Milliseconds
#define BIAS_DETECTION_TURNS 1000 // Get Moving Average of ADC Values at Start to Know Voltage Bias

typedef union _adc16 {
	struct _value8 {
		uint8_t lower; // Bit[7:0] = ADC[7:0]
		uint8_t upper; // Bit[1:0] = ADC[9:8]
	} value8;
	int16_t value16;
} adc16;

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint16_t input_sensitivity_count;
adc16 adc_bias;
uint8_t input_pin_last;
uint8_t input_pin_buffer;

int main(void) {
	/* Declare and Define Local Constants and Variables */
	adc16 adc_sample;
	adc16 adc_sample2;
	uint8_t const start_adc = _BV(ADSC);
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL

	/* Initialize Global Variables */
	input_sensitivity_count = INPUT_SENSITIVITY;
	input_pin_last = 0;
	input_pin_buffer = 0;

	/* Clock Calibration */
	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */
	PORTB = _BV(PB2)|_BV(PB1); // Pullup Button Input (There is No Internal Pulldown)
	DDRB = _BV(DDB0); // Bit Value Set PB0 (OC0A) as Output

	/* ADC */
	// For Noise Reduction of ADC, Disable Digital Input Buffers
	DIDR0 = _BV(ADC0D)|_BV(ADC3D)|_BV(ADC2D)|_BV(AIN0D);
	// Set ADC, Voltage Reference VCC, ADC2 (PB4)
	ADMUX = _BV(MUX1);
	// ADC Enable, Prescaler 16 to Have ADC Clock 600Khz
	ADCSRA = _BV(ADEN)|_BV(ADPS2);

	/* Counter/Timer */
	// Counter Reset
	TCNT0 = 0;
	// Set Output Compare A
	OCR0A = PWM_BIAS;
	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);
	// Select Fast PWM Mode (3) and Output from OC0A Non-inverted and OC0B Non-inverted
	// Fast PWM Mode (7) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM01)|_BV(WGM00)|_BV(COM0B1)|_BV(COM0A1);
	// Start Counter with I/O-Clock 9.6MHz / ( 1 * 256 ) = 37500Hz
	TCCR0B = _BV(CS00);

	/* Voltage Bias Detection */
	// Wait for End of Transient Response Since Power On or Reset
	_delay_ms( BIAS_DETECTION_DELAY );
	ADCSRA |= start_adc;
	while( ADCSRA & start_adc );
	adc_sample.value8.lower = ADCL;
	adc_sample.value8.upper = ADCH; // ADC[9:0] Will Be Updated After High Bits Are Read
	for ( uint16_t i = 0; i < BIAS_DETECTION_TURNS; i++ ) {
		ADCSRA |= start_adc;
		while( ADCSRA & start_adc );
		adc_sample2.value8.lower = ADCL;
		adc_sample2.value8.upper = ADCH;
		adc_sample.value16 = (adc_sample.value16 + adc_sample2.value16) >> 1;
		_delay_us( 30 );
	}
	adc_bias.value16 = adc_sample.value16 + ADC_BIAS_CORRECTION;

	/* Preperation to Enter Loop */
	// For First Obtention of ADC Value on Loop
	ADCSRA |= start_adc;
	// Counter Reset
	TCNT0 = 0;
	// Start to Issue Interrupt
	sei();

	while(1) {
	}
	return 0;
}

ISR(TIM0_OVF_vect, ISR_NAKED) { // No Need to Save Registers and SREG Before Entering ISR
	/* Declare and Define Local Constants and Variables */
	uint8_t const start_adc = _BV(ADSC);
	uint8_t const pin_input = _BV(PINB2)|_BV(PINB1); // Assign PB2 and PB1 as Gain Bit[1:0]
	uint8_t const pin_input_shift = PINB1;
	adc16 adc_sample;
	uint8_t input_pin;

	adc_sample.value8.lower = ADCL;
	adc_sample.value8.upper = ADCH;
	ADCSRA |= start_adc; // For Next Sampling

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

	//adc_sample.value16 -= ADC_BIAS_DEFAULT;
	adc_sample.value16 -= adc_bias.value16;
	// Arithmetic Left Shift (Signed Value in Bit[9:0], Bit[15:10] Same as Bit[9])
	adc_sample.value16 <<= input_pin_buffer; // Gain Bit[1:0]
	adc_sample.value16 += PWM_BIAS; // Gain 12dB (Multiplier 4)
	if ( adc_sample.value16 > PWM_CLIP_UPPER ) {
		adc_sample.value16 = PWM_CLIP_UPPER;
	}
	if ( adc_sample.value16 < PWM_CLIP_UNDER ) {
		adc_sample.value16 = PWM_CLIP_UNDER;
	}
	OCR0A = adc_sample.value8.lower;
	reti();
}