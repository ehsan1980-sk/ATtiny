/**
 * Copyright 2021 Kenta Ishii
 * License: 3-Clause BSD License
 * SPDX Short Identifier: BSD-3-Clause
 */

#define SEQUENCER_SAMPLE_RATE (double)(F_CPU / 510) // Approx. 31372.55 Samples per Seconds
#define SEQUENCER_PROGRAM_COUNTUPTO 32
#define SEQUENCER_PROGRAM_LENGTH 2 // Length of Sequence
#define SEQUENCER_BYTE_GROUP_BIT 0x50
#define SEQUENCER_BYTE_START_BIT 0x08
#define SEQUENCER_BYTE_GROUP_START_BIT (SEQUENCER_BYTE_GROUP_BIT|SEQUENCER_BYTE_START_BIT)
#define SEQUENCER_BYTE_PROGRAM_MASK 0x07

/* Global Variables without Initialization to Define at .bss Section and Squash .data Section */

volatile uint16_t sequencer_count_update;
volatile uint8_t sequencer_is_start;
volatile uint8_t sequencer_program_byte;

/**
 * Bit[7:0]: Data of Serial Communication (UART)
 */
uint8_t const sequencer_program_array[SEQUENCER_PROGRAM_LENGTH][SEQUENCER_PROGRAM_COUNTUPTO] PROGMEM = { // Array in Program Space
	{"Type X / Y to Get Transmission\r\n"}, // Sequence Index No. 0
	{"Type P to Reset Transmission--\r\n"} // Sequence Index No. 1
};
