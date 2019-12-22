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
 * Output Sawtooth Wave from PB0 (OC0A)
 * Output Triangle Wave from PB1 (OC0B), Frequency: Twice as Much as Sawtooth Wave
 * Input from PB2 (ADC1) to Determine Output Frequency
 * Input from PB4 (ADC2) to Determine Pitch of Output Frequency, ADC Value 0 Means -16, ADC Value 255 Means +15
 * Note: The wave may not reach the high peak, 0xFF (255) in default,
 *       because of its low precision decimal system.
 *       Especially, the lower frequency loses the high peak, e.g., 0.125Hz reaches up to 0xEF (239) through Round Off.
 */

#define SAMPLE_RATE (double)(F_CPU / 510 * 64) // Approx. 294.117647 Samples per Seconds
#define PEAK_LOW 0x00
#define PEAK_HIGH 0xFF
#define PEAK_TO_PEAK (PEAK_HIGH - PEAK_LOW)

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

uint16_t sample_count;
uint16_t count_per_2pi; // Count per 2Pi Radian

/**
 *                      SAMPLE_RATE
 * count_per_2pi + 1 = -------------
 *                       Frequency
 */

uint16_t fixed_value_sawtooth; // Fixed Point Arithmetic, Bit[15] Sign, Bit[14:7] UINT8, Bit[6:0] Fractional Part
uint16_t fixed_delta_sawtooth; // Fixed Point Arithmetic, Bit[15] Sign, Bit[14:7] UINT8, Bit[6:0] Fractional Part
uint16_t fixed_value_triangle; // Fixed Point Arithmetic, Bit[15] Sign, Bit[14:7] UINT8, Bit[6:0] Fractional Part
uint8_t toggle_triangle;

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
	uint8_t value_adc_channel_1_high_buffer; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_1_high = 0; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_2_high_buffer; // Bit[7:0] Is ADC[9:2]
	uint8_t value_adc_channel_2_high = 0; // Bit[7:0] Is ADC[9:2]
	uint16_t count_per_2pi_buffer;
	uint16_t fixed_delta_sawtooth_buffer;
	uint8_t osccal_default; // Calibrated Default Value of OSCCAL
	int8_t osccal_tuning = 0; // Tuning Value for Variable Tone
	int8_t osccal_pitch = 0; // Pitch Value from ADC2
	int8_t osccal_pitch_buffer;

	/* Initialize Global Variables */

	count_per_2pi = 0;

	/* Clock Calibration */

	osccal_default = OSCCAL + CALIB_OSCCAL; // Frequency Calibration for Individual Difference at VCC = 3.3V
	OSCCAL = osccal_default;

	/* I/O Settings */

	PORTB = 0; // All Low
	DDRB = _BV(DDB1)|_BV(DDB0); // Bit Value Set PB0 (OC0A) and PB1 (OC0B) as Output

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
	OCR0A = PEAK_LOW;

	// Set Output Compare B
	OCR0B = PEAK_LOW;

	// Set Timer/Counter0 Overflow Interrupt for "ISR(TIM0_OVF_vect)"
	TIMSK0 = _BV(TOIE0);

	// Select PWM (Phase Correct) Mode (1) and Output from OC0A Non-inverted and OC0B Non-inverted
	// PWM (Phase Correct) Mode (5) can make variable frequencies with adjustable duty cycle by settting OCR0A as TOP, but OC0B is only available.
	TCCR0A = _BV(WGM00)|_BV(COM0B1)|_BV(COM0A1);

	// Start Counter with I/O-Clock 9.6MHz / ( 510 * 64 ) = Approx. 294.117647Hz
	TCCR0B = _BV(CS00)|_BV(CS01);

	// Start to Issue Interrupt
	sei();

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
				fixed_delta_sawtooth_buffer = 7<<7|0b0001010;
				osccal_tuning = 1;
			} else if ( value_adc_channel_1_high >= 192 ) {
				count_per_2pi_buffer = 72; // 4 Hz
				fixed_delta_sawtooth_buffer = 3<<7|0b1000101;
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 160 ) {
				count_per_2pi_buffer = 146; // 2 Hz
				fixed_delta_sawtooth_buffer = 1<<7|0b1011111;
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 128 ) {
				count_per_2pi_buffer = 293; // 1 Hz
				fixed_delta_sawtooth_buffer = 0<<7|0b1101111;
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 96 ) {
				count_per_2pi_buffer = 587; // 0.5 Hz
				fixed_delta_sawtooth_buffer = 0<<7|0b0110111;
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 64 ) {
				count_per_2pi_buffer = 1175; // 0.25 Hz
				fixed_delta_sawtooth_buffer = 0<<7|0b0011011;
				osccal_tuning = 0;
			} else if ( value_adc_channel_1_high >= 32 ) {
				count_per_2pi_buffer = 2352; // 0.125 Hz
				fixed_delta_sawtooth_buffer = 0<<7|0b0001101;
				osccal_tuning = 0;
			} else { // ADC Value < 32
				count_per_2pi_buffer = 0;
				fixed_delta_sawtooth_buffer = 0;
				osccal_tuning = 0;
			}
			if ( count_per_2pi_buffer != count_per_2pi ) {
				if ( count_per_2pi_buffer ) {
					// Fixed Point Arithmetic (DIV), LSL7 to Dividend, Needed UINT32
					//fixed_delta_sawtooth_buffer = ((PEAK_TO_PEAK << 7) << 7) / (count_per_2pi_buffer << 7);
					cli(); // Stop to Issue Interrupt
					sample_count = 0;
					toggle_triangle = 0;
					count_per_2pi = count_per_2pi_buffer;
					fixed_delta_sawtooth = fixed_delta_sawtooth_buffer;
					function_start = 1;
					sei(); // Start to Issue Interrupt
				} else {
					cli();  // Stop to Issue Interrupt
					function_start = 0;
					OCR0A = PEAK_LOW;
					OCR0B = PEAK_LOW;
					sei(); // Start to Issue Interrupt
				}
				OSCCAL = osccal_default + osccal_tuning + osccal_pitch;
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
			osccal_pitch_buffer = ((int8_t)(0x80^(value_adc_channel_2_high)) >> 3);
			if ( osccal_pitch_buffer !=  osccal_pitch ) {
				osccal_pitch = osccal_pitch_buffer;
				OSCCAL = osccal_default + osccal_tuning + osccal_pitch;
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
			fixed_value_sawtooth = PEAK_LOW << 7;
		} else if ( sample_count <= count_per_2pi ) {
			/* Equivalence of */
			// Fixed Point Arithmetic (MUL), LSR7 after Multiplication, Needed UINT32
			//fixed_value_sawtooth = ((fixed_delta_sawtooth * (sample_count << 7)) >> 7) + (PEAK_LOW << 7);
			/* And */
			fixed_value_sawtooth += fixed_delta_sawtooth; // Fixed Point Arithmetic (ADD)
			/* End of Equivalence */
			temp = (fixed_value_sawtooth << 1) >> 8; // Make Bit[7:0] UINT8 (Considered of Clock Cycle)
			if ( 0x0040 & fixed_value_sawtooth ) temp++; // Check Fractional Part Bit[6] (0.5) to Round Off
			OCR0A = temp;
		}
		// Triangle Wave
		if ( ! toggle_triangle ) { // Increment
			if ( sample_count == 0 ) {
				OCR0B = PEAK_LOW;
				fixed_value_triangle = PEAK_LOW << 7;
			} else if ( sample_count <= count_per_2pi ) {
				fixed_value_triangle += fixed_delta_sawtooth; // Fixed Point Arithmetic (ADD)
				temp = (fixed_value_triangle << 1) >> 8; // Make Bit[7:0] UINT8 (Considered of Clock Cycle)
				if ( 0x0040 & fixed_value_triangle ) temp++; // Check Fractional Part Bit[6] (0.5) to Round Off
				OCR0B = temp;
			}
		} else {
			if ( sample_count == 0 ) { // Decrement
				OCR0B = fixed_value_triangle;
			} else if ( sample_count <= count_per_2pi ) {
				fixed_value_triangle -= fixed_delta_sawtooth; // Fixed Point Arithmetic (SUB)
				temp = (fixed_value_triangle << 1) >> 8; // Make Bit[7:0] UINT8 (Considered of Clock Cycle)
				if ( 0x0040 & fixed_value_triangle ) temp++; // Check Fractional Part Bit[6] (0.5) to Round Off
				OCR0B = temp;
			}

		}
		sample_count++;
		if ( sample_count > count_per_2pi ) {
			sample_count = 0;
			toggle_triangle ^= 1;
		}
	}
}
