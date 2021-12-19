/**
 * Copyright 2021 Kenta Ishii
 * License: 3-Clause BSD License
 * SPDX Short Identifier: BSD-3-Clause
 */

#define SEQUENCER_VOLTAGE_BIAS 0x80 // Decimal 128 on Noise Off
#define SEQUENCER_SAMPLE_RATE (double)(F_CPU / 256) // 31250 Samples per Seconds
#define SEQUENCER_INTERVAL_NUMBER 9
#define SEQUENCER_INTERVAL_INDEX_DEFAULT 0
#define SEQUENCER_PROGRAM_COUNTUPTO 64
#define SEQUENCER_PROGRAM_LENGTH 2 // Length of Sequence
#define SEQUENCER_LEVEL_SHIFT_MAX 3
#define SEQUENCER_INPUT_SENSITIVITY 250 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)
#define SEQUENCER_BUTTON_SENSITIVITY 2500 // Less Number, More Sensitive (Except 0: Lowest Sensitivity)

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

volatile uint16_t sequencer_interval_max;
volatile uint16_t sequencer_interval_count;
volatile uint16_t sequencer_count_update;
volatile uint16_t sequencer_interval_random;
volatile uint16_t sequencer_interval_random_max;
volatile uint8_t sequencer_next_random;

// Interval (31250 Divided by Beats in 1 Second)
uint16_t const sequencer_interval_array[SEQUENCER_INTERVAL_NUMBER] PROGMEM = { // Array in Program Space
	3906, // 8 Beats
	3472, // 9 Beats
	3125, // 10 Beats
	2841, // 11 Beats
	2604, // 12 Beats
	2404, // 13 Beats
	2232, // 14 Beats
	2083, // 15 Beats
	1953 // 16 Beats
};

// Delay Time in Turns to Generate Next Random Value
uint16_t const sequencer_interval_random_max_array[16] PROGMEM = { // Array in Program Space
	1,
	2,
	3,
	4,
	8,
	16,
	32,
	64,
	128,
	192,
	256,
	384,
	512,
	768,
	1024,
	1536
};

uint8_t const sequencer_volume_mask_array[8] PROGMEM = { // Array in Program Space
	0x00,
	0x07, // Up to Decimal 7
	0x0F, // Up to Decimal 15
	0x1F, // Up to Decimal 31
	0x3F, // Up to Decimal 63
	0x7F, // Up to Decimal 127
	0xBF, // Up to Decimal 191
	0xFF // Up to Decimal 255
};

uint8_t const sequencer_volume_offset_array[8] PROGMEM = { // Array in Program Space
	SEQUENCER_VOLTAGE_BIAS,
	0x7C, // Decimal 124
	0x78, // Decimal 120
	0x70, // Decimal 112
	0x60, // Decimal 96
	0x40, // Decimal 64
	0x20, // Decimal 32
	0x00 // Decimal 0
};

/**
 * Bit[3:0]: Index of sequencer_interval_random_max_array (0-15)
 * Bit[6:4]: Index of sequencer_volume_mask_array and sequencer_volume_offset_array (0-7)
 * Bit[7]: 0 as 7-bit LFSR-2, 1 as 15-bit LFSR-2
 */
uint8_t const sequencer_program_array[SEQUENCER_PROGRAM_LENGTH][SEQUENCER_PROGRAM_COUNTUPTO] PROGMEM = { // Array in Program Space
	{0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,
	 0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,
	 0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,
	 0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00,0x70,0x00,0x00,0x00,0x71,0x00,0x00,0x00}, // Sequence Index No. 0
	{0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xF0,0xF5,0xF0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,
	 0xF5,0xA5,0xF0,0xA0,0xF5,0xA5,0xF0,0xA0,0xF5,0xF0,0xF5,0xF0,0xF5,0xF0,0xF5,0xF0} // Sequence Index No. 1
};
