#include <stdint.h>



// CUSTOM LOGICS
#define RX_MODE

#define RX_BIT_PACKET_SIZE_DEBUG 1
static uint8_t shift_reg = 0;


#define PREAMBLE_INTERVAL_SAMPLES (TX_SAMPLE_RATE * 1)
#define WINDOW RX_BIT_SAMPLES

static uint64_t best_energy = 0;
static uint32_t best_offset = 0;
static uint32_t current_offset = 0;
// output methods
// #define DECODE_PRINT_USB_BATCH
// #define DECODE_PRINT_UART_EACH
// #define DECODE_PRINT_UART_BATCH

// RADIO PARAMETERS
#define UART_WORKING
#ifdef UART_WORKING
	#define TX_SAMPLE_RATE      5000000
	#define RX_SAMPLE_RATE      5000000
	#define CENTRE_FREQ         915000000
	#define SINE_FREQ           100000
	#define TX_BIT_SAMPLES      1024
	#define RX_BIT_SAMPLES      1024 /* Less bit samples means more transfer overhead! */
	#define RX_BIT_SHIFT 10 	// 1024 -> 10
	#define RX_THRESHOLD_MARGIN 50
	#define TX_RF_GAIN          30 // 30
	#define TX_IF_GAIN          47 // 47
	#define RX_IF_GAIN          36
	#define RX_RF_GAIN          36
	#define RX_BB_GAIN          36
	#define TX_ANTENNA_ENABLE   false
	#define RX_ANTENNA_ENABLE   false
#endif

#ifdef USB_STOCK_WORKING
	#define TX_SAMPLE_RATE      5000000
	#define RX_SAMPLE_RATE      5000000
	#define CENTRE_FREQ         915000000
	#define SINE_FREQ           100000
	#define TX_BIT_SAMPLES      10000
	#define RX_BIT_SAMPLES      10000 /* Less bit samples means more transfer overhead! */
	#define RX_THRESHOLD_MARGIN 20
	#define TX_RF_GAIN          30
	#define TX_IF_GAIN          47
	#define RX_IF_GAIN          32
	#define RX_RF_GAIN          30
	#define RX_BB_GAIN          30
	#define TX_ANTENNA_ENABLE   false
	#define RX_ANTENNA_ENABLE   false
#endif

// USB_TRANSFER SIZE
#if defined CUSTOM_TX_MODE || defined CUSTOM_RX_MODE
	#define BATCH_SAMPLE_SIZE 256
#endif
#ifndef BATCH_SAMPLE_SIZE
	#define BATCH_SAMPLE_SIZE  128//0x4000 /* < Original Firmware */
#endif

#ifndef RX_THRESHOLD_MARGIN
	#define RX_THRESHOLD_MARGIN 0
#endif

#define USB_TRANSFER_SIZE 0x4000

// BIT PATTERN SIZE
#define TX_PATTERN_MAX_BITS 64U

// RING CONFIGURATION
#define N 2

// ================= CONFIG =================

// Device ID (change per node)
static const uint8_t mac_device_id = 0x00;

// ================= MAC-PROTOCOL =================
#define MAC_HEADER_SIZE      (3+1+1+1+3)      // preamble, TX, RX, nextTX, payloadsize (in Bytes)
#define MAC_MAX_PAYLOAD      (256) // MAX Bytes of data
#define MAC_MAX_FRAME_SIZE   (MAC_HEADER_SIZE + MAC_MAX_PAYLOAD) // in Bytes
#define MAC_MAX_FRAME_SIZE_BITS         (MAC_MAX_FRAME_SIZE*8) // in BITS

// If we decide to add CRC later 
//#define MAC_CRC_SIZE         2
//#define MAC_MAX_FRAME_SIZE   (MAC_HEADER_SIZE + MAC_MAX_PAYLOAD + MAC_CRC_SIZE)
//#define MAC_MAX_FRAME_SIZE_BITS         (MAC_MAX_FRAME_SIZE*8)

// ================= PREAMBLE =================

static const uint8_t PREAMBLE16[] = {
    1,0,1,0,1,0,1,0,
    1,0,1,0,1,0,1,0,
    1,0,0,1,1,0,0,1
};

#define PREAMBLE_BITS (sizeof(PREAMBLE16))

typedef struct {
    uint8_t data[MAC_MAX_FRAME_SIZE];
    uint32_t len;
} packet_t;

// ================= DATA =================

// BIT PATTERN TEST
// #define CUSTOM_BIT_PATTERN {1, 0, 1, 0, 1, 0, 1, 0}
// #define CUSTOM_BIT_PATTERN {1, 0, 1, 1, 0, 1, 1, 1, 0}
//#define CUSTOM_BYTE_PATTERN {1, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0}
// define CUSTOM_BIT_PATTERN {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,1,1,0,0,1, 0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,1 ,0,0,0,0,0,0,0,1,	0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,	  1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 1}
#define CUSTOM_BYTE_PATTERN {0x9B, 0x8D, 0xD2, 0xCE, 0x58, 0xB9}

// size = 0,0,26 hexadecimal, no TX,RX,next_TX, this will be added by the MAC protocol layer
/* 
#define CUSTOM_BYTE_PATTERN { \
	0x00,0x00,0x1A,0x3E,0x91, \ 
	0x4B,0x6D,0xF0,0x22,0x9A,0xC7,0x11,0xE5, \
	0x3C,0x08,0xB6,0xD1,0x55,0xAF,0x90,0x12, \
	0x6E,0xF3,0x4A,0x81,0xC0,0x39,0x7D,0x2B \
	}
 */