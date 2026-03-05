// sine_table.h
#ifndef SINE_TABLE_H
#define SINE_TABLE_H

#include <stdint.h>

#define SINE_TABLE_SIZE 256
#define SINE_AMPLITUDE 80

extern uint8_t sine_table[SINE_TABLE_SIZE * 2];  // IQ samples

void init_sine_table(void);

#endif