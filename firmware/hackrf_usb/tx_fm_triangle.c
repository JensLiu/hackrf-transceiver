// #include <stdint.h>
// #include <stdbool.h>
// #include <math.h>
// #include "hackrf_core.h"
// #include "usb_queue.h"
// #include "gpio.h"
// #include "max2837.h"
// #include "si5351c.h"
// #include "sgpio.h"
// #include "usb_api_transceiver.h"

// #include "max2837.h"            // MAX2837 transceiver driver (for gain settings)
// #include "rf_path.h"            // RF path controls (antenna, amplifiers, direction)
// #include "sgpio.h"              // SGPIO/DMA configuration

// #define SAMPLE_RATE         10000000  // 10 MSPS
// #define FM_MOD_FREQ         440       // Triangle wave modulating frequency (Hz)
// #define BUFFER_LEN          512       // IQ samples per buffer
// #define FM_DEVIATION        3000      // Hz deviation for FM
// #define TX_SECONDS          5         // Duration to transmit

// #define FIXED_PI            205887     // PI * (1 << 16)
// #define PHASE_INC(freq)     ((uint32_t)((freq * 65536.0) / SAMPLE_RATE))

// extern usb_queue_t tx_queue;  // already declared in hackrf_usb.c

// static uint8_t tx_buffer[BUFFER_LEN * 2]; // I/Q interleaved, 8-bit signed samples

// void tx_fm_triangle_start() {
//     uint32_t samples_to_send = SAMPLE_RATE * TX_SECONDS;
//     uint32_t samples_sent = 0;

//     // Triangle wave state
//     int32_t triangle = -32767;
//     int32_t triangle_step = (4 * 32767 * FM_MOD_FREQ) / SAMPLE_RATE;  // Full swing in one cycle
//     int8_t direction = 1;

//     // Phase accumulator
//     uint32_t phase = 0;

//     max2837_init();                          // Initialize the RF front-end
//     max2837_set_tx();                        // Set MAX2837 to TX mode

//     si5351c_set_freq(0, 144500000);          // Set clock to 144.5 MHz
//     si5351c_enable_output();                 // Enable clock output

//     sgpio_configure_tx(SAMPLE_RATE);         // Configure SGPIO to transmit at desired sample rate

//     usb_queue_init(&tx_queue);              // Initialize transmit queue

//     sgpio_enable();                          // Start SGPIO streaming (DMA)


//     while (samples_sent < samples_to_send) {
//         for (int i = 0; i < BUFFER_LEN; i++) {
//             // Update triangle wave
//             triangle += triangle_step * direction;
//             if (triangle >= 32767) {
//                 triangle = 32767;
//                 direction = -1;
//             } else if (triangle <= -32767) {
//                 triangle = -32767;
//                 direction = 1;
//             }

//             // Frequency deviation mapping: map triangle [-32767, 32767] to [-FM_DEVIATION, FM_DEVIATION]
//             int32_t freq_dev = (triangle * FM_DEVIATION) / 32767;
//             phase += PHASE_INC(144500000 + freq_dev);  // FM modulation

//             // Generate IQ sample (simple sin/cos approximation)
//             float phase_rad = (2.0f * M_PI * phase) / 65536.0f;
//             int8_t i_sample = (int8_t)(127 * cosf(phase_rad));
//             int8_t q_sample = (int8_t)(127 * sinf(phase_rad));

//             tx_buffer[2 * i] = i_sample;
//             tx_buffer[2 * i + 1] = q_sample;
//         }

//         tx_submit_buffer(tx_buffer, sizeof(tx_buffer));
//         samples_sent += BUFFER_LEN;
//     }

//     sgpio_disable();                         // Stop SGPIO streaming
//     max2837_set_rx();                        // Optionally switch back to RX mode
// }
