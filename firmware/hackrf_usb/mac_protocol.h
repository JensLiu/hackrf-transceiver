#ifndef MAC_PROTOCOL_H
#define MAC_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>
#include "mac_protocol.h"

// ================= CONFIG =================

// ================= MAC LIMITS =================

#define MAC_HEADER_SIZE            (3 + 1 + 1 + 1 + 3)   // bytes
#define MAC_MAX_PAYLOAD            (256)                 // bytes
#define MAC_MAX_FRAME_SIZE         (MAC_HEADER_SIZE + MAC_MAX_PAYLOAD)
#define MAC_MAX_FRAME_SIZE_BITS    (MAC_MAX_FRAME_SIZE * 8)

// ================= FRAME STRUCT =================

typedef struct {
    uint8_t tx;
    uint8_t rx;
    uint8_t next_tx;
    uint32_t payload_size;              // NOTE: 24-bit field stored in 32-bit
    uint8_t payload[MAC_MAX_PAYLOAD];
} mac_frame_t;

// ================= TX API =================

// Build a frame and prepare internal bit buffer
void mac_build_frame(uint8_t rx,
                     uint8_t next_tx,
                     uint8_t payload_size,
                     uint8_t* payload
                     );

// Get pointer to bitstream for PHY transmission
bool* mac_get_tx_bits(uint32_t* len);

bool* mac_get_tx_data(uint32_t* len);

// ================= RX API =================

// Reset RX state machine
void mac_reset_rx(void);

// Bit-wise processing (if PHY outputs bits)
bool mac_process_bit(uint8_t bit, mac_frame_t* out_frame);

// Byte-wise processing (recommended for performance)
bool mac_process_byte(uint8_t byte, mac_frame_t* out_frame);


#endif // MAC_PROTOCOL_H