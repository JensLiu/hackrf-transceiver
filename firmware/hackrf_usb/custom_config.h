// CUSTOM LOGICS
#define CUSTOM_RX_MODE

// BIT PATTERN TEST
// #define CUSTOM_BIT_PATTERN {1, 0, 1, 0, 1, 0, 1, 0}
#define CUSTOM_BIT_PATTERN {1, 0, 1, 1, 0, 1, 1, 1, 0}
// #define CUSTOM_BIT_PATTERN {1, 0, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0}
#define RX_BIT_PACKET_SIZE 12

// RADIO PARAMETERS
#define UART_WORKING
#ifdef UART_WORKING
	#define TX_SAMPLE_RATE      5000000
	#define RX_SAMPLE_RATE      5000000
	#define CENTRE_FREQ         915000000
	#define SINE_FREQ           100000
	#define TX_BIT_SAMPLES      1000
	#define RX_BIT_SAMPLES      1000 /* Less bit samples means more transfer overhead! */
	#define RX_THRESHOLD_MARGIN 20
	#define TX_RF_GAIN          30
	#define TX_IF_GAIN          47
	#define RX_IF_GAIN          32
	#define RX_RF_GAIN          30
	#define RX_BB_GAIN          30
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
	#define USB_TRANSFER_SIZE 256
#endif
#ifndef USB_TRANSFER_SIZE
	#define USB_TRANSFER_SIZE 0x4000 // < Original Firmware
#endif

#ifndef RX_THRESHOLD_MARGIN
	#define RX_THRESHOLD_MARGIN 0
#endif

// BIT PATTERN SIZE
#define TX_PATTERN_MAX_BITS 64U