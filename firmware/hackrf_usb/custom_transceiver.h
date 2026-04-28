#include "hackrf_core.h"
#include "streaming.h"
#include "m0_state.h"
#include "usb_bulk_buffer.h"
#include "tuning.h"

void custom_transceiver_receive_init();
void custom_transceiver_receive_begin();
void custom_transceiver_receive();
void custom_transceiver_receive_end();

void rx_mode();