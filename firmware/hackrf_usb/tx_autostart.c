// tx_autostart.c

#include <stdint.h>
#include <string.h>
#include "usb_api_transceiver.h"
#include "hackrf_core.h"
#include "max2837.h"
#include "si5351c.h"

#include "streaming.h"
#include "tuning.h"
#include "max2837.h"
#include "usb_api_transceiver.h"
#include <string.h>
#include "rf_path.h"
#include "usb_bulk_buffer.h"

#define SAMPLE_RATE_HZ      2000000
#define TX_FREQ_HZ     915000000ULL
#define TX_VGA_GAIN         30

uint32_t offset = 0;
uint32_t block_size = 0x4000;  // 16 KB per USB transfer block

/*uint8_t* block = &usb_bulk_buffer[offset & USB_BULK_BUFFER_MASK];

extern max2837_driver_t max2837;

// Fill the buffer with alternating 0x7FFF and 0x8001 (representing IQ = 1, -1)
sample_t* tx_sample_callback(void) {
    for (int i = 0; i < BUFFER_SIZE; i += 2) {
        tx_buffer[i] = 127;      // I
        tx_buffer[i+1] = 127;    // Q
    }

    return tx_buffer;
}

// This function is called from main() at boot to configure TX
void tx_autostart_init(void) {
    // Set frequency (Si5351C, RFFC5071, MAX2837)
    set_freq(TX_FREQ_HZ);

    // Set sample rate
    sample_rate_frac_set(SAMPLE_RATE_HZ, 1750000); // example divider

    // Set gain
    max2837_set_txvga_gain(&max2837, TX_VGA_GAIN);

    // Enable amplifier
    rf_path_set_lna(&rf_path, 1);

    // Register our custom buffer fill function
    sample_tx_callback = tx_sample_callback;

    // Start transmitting
    transceiver_start(TRANSCEIVER_MODE_TX);
}
*/