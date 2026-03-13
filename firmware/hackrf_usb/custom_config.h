// CUSTOM LOGICS
#define CUSTOM_RX_MODE

// #define CUSTOM_BIT_PATTERN {1, 0, 1, 1, 0, 1, 1, 1, 0}
#define CUSTOM_BIT_PATTERN {1, 0, 1, 0, 1, 0, 1, 0}

#define RX_BIT_PACKET_SIZE 4

// RADIO PARAMETERS
#define IN_PROGRESS
#ifdef IN_PROGRESS
	#define TX_SAMPLE_RATE 5000000
	#define RX_SAMPLE_RATE 5000000
	#define CENTRE_FREQ    915000000
	#define SINE_FREQ      100000
	// #define TX_BIT_SAMPLES      1000
	// #define RX_BIT_SAMPLES      200 /* < needs adjusting */
	#define RX_BIT_SAMPLES      20000000
	#define TX_BIT_SAMPLES      10000000
	#define RX_THRESHOLD_MARGIN 20
	#define TX_RF_GAIN          30
	#define TX_IF_GAIN          47
	#define RX_IF_GAIN          40 /* < needs adjusting */
	#define RX_RF_GAIN          30 /* < needs adjusting */
	#define TX_ANTENNA_ENABLE   false
	#define RX_ANTENNA_ENABLE   false
#else
// Porting Ama's code
	#define SAMPLE_RATE         10000000
	#define SINE_FREQ           200000
	#define CENTRE_FREQ         915000000
	#define TX_BIT_SAMPLES      1000
	#define RX_BIT_SAMPLES      2000
	#define RX_THRESHOLD_MARGIN 20
	#define RX_IF_GAIN          40
	#define RX_RF_GAIN          14
	#define TX_IF_GAIN          47
	#define TX_RF_GAIN          14
	#define TX_ANTENNA_ENABLE   true
	#define RX_ANTENNA_ENABLE   true
#endif

// USB_TRANSFER SIZE
#if defined CUSTOM_TX_MODE || defined CUSTOM_RX_MODE
	#define USB_TRANSFER_SIZE 256
#endif
#ifndef USB_TRANSFER_SIZE
	#define USB_TRANSFER_SIZE 0x4000 // < Original Firmware
#endif

// BIT PATTERN SIZE
#define TX_PATTERN_MAX_BITS 64U