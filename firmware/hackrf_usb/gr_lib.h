#include <hackrf_core.h>
#include <usb_type.h>
#include <usb_request.h>

// MAPPING FROM THE GR-OSMOSDR PROJECT
void rx_set_if_gain(uint32_t);
void rx_set_rf_gain(uint32_t);
void rx_set_bb_gain(double);
void tx_set_rf_gain(uint32_t);
void tx_set_if_gain(double);
void set_baseband_filter_bandwidth(double);
int set_sample_rate(double);
void set_centre_frequency(double);
void set_antenna_enable(bool);