
#include "custom_config.h"
#include "usb_api_transceiver.h"
#include "mac_protocol.h"


#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ================= INTERNAL BUFFERS =================

// Bit stream used by PHY
static bool mac_tx_bits[MAC_MAX_FRAME_SIZE_BITS];
static uint8_t mac_tx_data[MAC_MAX_FRAME_SIZE];

static uint32_t mac_tx_bits_len = 0;
static uint32_t mac_tx_data_len = 0;

// RX reconstruction
static uint8_t mac_rx_bytes[MAC_MAX_FRAME_SIZE];
static uint32_t mac_rx_byte_index = 0;
static uint8_t mac_rx_bit_accumulator = 0;
static uint8_t mac_rx_bit_count = 0;

uint32_t payload_size = 0;

// ================= UTIL =================

// Convert byte → bits (MSB first)
static void byte_to_bits(uint8_t byte, uint8_t* out_bits, uint32_t* index)
{
    for (int i = 7; i >= 0; i--) {
        out_bits[(*index)++] = (byte >> i) & 1;
    }
}

// Convert bits → byte
static uint8_t bits_to_byte(uint8_t* bits)
{
    uint8_t val = 0;
    for (int i = 0; i < 8; i++) {
        val = (val << 1) | bits[i];
    }
    return val;
}

// ================= TX =================

static void mac_pack_bits_to_bytes(void)
{
    mac_tx_data_len = (mac_tx_bits_len + 7) / 8;
    memset(mac_tx_data, 0, mac_tx_data_len);

    for (uint32_t i = 0; i < mac_tx_bits_len; i++) {
        if (mac_tx_bits[i]) {
            mac_tx_data[i / 8] |= (1u << (7 - (i % 8)));   // MSB-first
        }
    }
}

bool* mac_get_tx_bits(uint32_t* len)
{
    if (len != NULL) {
        *len = mac_tx_bits_len;
    }
    return mac_tx_bits;
}

uint8_t* mac_get_tx_data(uint32_t* len)
{
    if (len != NULL) {
        *len = mac_tx_data_len;
    }
    return mac_tx_data;
}


void mac_build_frame(uint8_t rx,
                     uint8_t next_tx,
                     uint32_t payload_size,
                     uint8_t* payload
                     )
{
    mac_tx_bits_len = 0;

    // 1. Add PREAMBLE (bit level)
    for (uint32_t i = 0; i < PREAMBLE_BITS; i++) {
        mac_tx_bits[mac_tx_bits_len++] = PREAMBLE16[i];
    }

    // 2. Header fields (byte → bits) 
    byte_to_bits(mac_device_id, mac_tx_bits, &mac_tx_bits_len);
    byte_to_bits(rx,            mac_tx_bits, &mac_tx_bits_len);
    byte_to_bits(next_tx,       mac_tx_bits, &mac_tx_bits_len);
    byte_to_bits((payload_size >> 16) & 0xFF, mac_tx_bits, &mac_tx_bits_len);
    byte_to_bits((payload_size >> 8)  & 0xFF, mac_tx_bits, &mac_tx_bits_len);
    byte_to_bits((payload_size)       & 0xFF, mac_tx_bits, &mac_tx_bits_len);

    // 3. Payload
    for (uint32_t i = 0; i < payload_size; i++) {
        byte_to_bits(payload[i], mac_tx_bits, &mac_tx_bits_len);
    }


    // Pack bits into bytes
    mac_pack_bits_to_bytes();

}

// ================= RX =================

void mac_reset_rx(void)
{
    mac_rx_byte_index = 0;
    mac_rx_bit_count = 0;
    mac_rx_bit_accumulator = 0;


}

/* // Call this for EVERY decoded bit from your RX
bool mac_process_bit(uint8_t bit, mac_frame_t* out_frame)
{
    // Accumulate bits into byte
    mac_rx_bit_accumulator = (mac_rx_bit_accumulator << 1) | bit;
    mac_rx_bit_count++;

    if (mac_rx_bit_count == 8) {
        mac_rx_bytes[mac_rx_byte_index++] = mac_rx_bit_accumulator;

        mac_rx_bit_count = 0;
        mac_rx_bit_accumulator = 0;

        // Minimum header received?
        if (mac_rx_byte_index >= 4) {

            uint8_t payload_size = mac_rx_bytes[3];

            if (mac_rx_byte_index >= (4 + payload_size)) {

                // Frame complete → parse
                out_frame->tx = mac_rx_bytes[0];
                out_frame->rx = mac_rx_bytes[1];
                out_frame->next_tx = mac_rx_bytes[2];
                out_frame->payload_size = payload_size;

                memcpy(out_frame->payload,
                       &mac_rx_bytes[4],
                       payload_size);

                mac_reset_rx();
                return true;
            }
        }
    }

    return false;
}
 */

bool mac_process_byte(uint8_t byte, mac_frame_t* out_frame)
{
    mac_rx_bytes[mac_rx_byte_index++] = byte;

    // Wait until full header (6 bytes, without preamble)
    if (mac_rx_byte_index == 6) {
        out_frame->tx = mac_rx_bytes[0];
        out_frame->rx = mac_rx_bytes[1];
        out_frame->next_tx = mac_rx_bytes[2];

        payload_size =
            ((uint32_t)mac_rx_bytes[3] << 16) |
            ((uint32_t)mac_rx_bytes[4] << 8)  |
            ((uint32_t)mac_rx_bytes[5]);
            
        out_frame->payload_size = payload_size;
        // Safety check (VERY important)
        if (payload_size > MAC_MAX_PAYLOAD) {
            mac_reset_rx();  // invalid frame
            return false;
        }

    }
    if (mac_rx_byte_index == (6 + payload_size)) {



        memcpy(out_frame->payload,
               &mac_rx_bytes[6],
               payload_size);

        mac_reset_rx();
        return true;
    }

    return false;
}