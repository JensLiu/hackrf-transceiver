// sine_table.c
#include "sine_table.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

uint8_t sine_table[SINE_TABLE_SIZE * 2];  // IQ samples

void init_sine_table(void) {
    for (int i = 0; i < SINE_TABLE_SIZE; i++) {
        // Calculate phase angle for this sample
        float angle = 2.0f * M_PI * i / SINE_TABLE_SIZE;
        
        // Generate I/Q samples with proper amplitude scaling
        // Scale to -SINE_AMPLITUDE to +SINE_AMPLITUDE range
        int8_t i_sample = (int8_t)(SINE_AMPLITUDE * sinf(angle));
        int8_t q_sample = (int8_t)(SINE_AMPLITUDE * cosf(angle));
        
        // Store as signed values (bit-preserving cast, NO +128 DC offset)
        // HackRF baseband uses signed 8-bit IQ samples
        sine_table[i * 2] = (uint8_t)i_sample;      // I sample
        sine_table[i * 2 + 1] = (uint8_t)q_sample;  // Q sample
    }
}

// void init_sine_table(void) {
//     for (int i = 0; i < SINE_TABLE_SIZE; i++) {
//         // Calculate phase angle for this sample
//         float angle = 2.0f * M_PI * i / SINE_TABLE_SIZE;
        
//         // Generate I/Q samples with proper amplitude scaling
//         // Scale to -SINE_AMPLITUDE to +SINE_AMPLITUDE range
//         int8_t i_sample = (int8_t)(SINE_AMPLITUDE * sinf(angle));
//         int8_t q_sample = (int8_t)(SINE_AMPLITUDE * cosf(angle));
        
//         // Convert to unsigned 8-bit by adding 128 (to center around 128)
//         sine_table[i * 2] = (uint8_t)(i_sample + 128);      // I sample
//         sine_table[i * 2 + 1] = (uint8_t)(q_sample + 128);  // Q sample
//     }
// }