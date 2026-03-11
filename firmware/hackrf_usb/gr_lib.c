#include "gr_lib.h"
#include "hackrf_ui.h"
#include "operacake_sctimer.h"

#include <libopencm3/cm3/vector.h>
#include "max2837.h"
#include "max2839.h"
#include "rf_path.h"
#include "tuning.h"
#include "streaming.h"
#include "usb.h"
#include "usb_queue.h"
#include "platform_detect.h"
#include <stddef.h>
#include "uart.h"
#include <math.h>
#include <unistd.h>

// MAPPING FROM THE GR-OSMOSDR PROJECT
void rx_set_if_gain(const uint32_t gain)
{
	//	if ( "IF" == name ) {
	//		return osmosdr::gain_range_t( 0, 40, 8 );
	//	}
	//	in-between ranges, clip to nearest
	const uint32_t clip_gain = (gain >= 40) ? 40 : ((uint32_t) (round(gain / 8))) * 8;
	const uint8_t value = (uint8_t) clip_gain;
	uart_printf("set rx if gain: %d -> %d\n", gain, value);
	if (RADIO_OK !=
	    radio_set_gain(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_GAIN_RX_LNA,
		    (radio_gain_t) {.db = value})) {
		uart_printf("standalone RX setup failed: IF gain\n");
	}
}

void rx_set_rf_gain(const uint32_t gain)
{
	//	if ( "RF" == name ) {
	//		return osmosdr::gain_range_t( 0, 14, 14 );
	//	}
	const uint8_t value = (gain >= 14) ? 1 : 0;
	uart_printf("set rx rf gain: %d -> value=%u\n", gain, value);
	if (RADIO_OK !=
	    radio_set_gain(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_GAIN_RF_AMP,
		    (radio_gain_t) {.enable = value})) {
		uart_printf("standalone RX setup failed: RF Gain\n");
	}
}

void rx_set_bb_gain(const double gain)
{
	//	if ("BB" == name) {
	//		return osmosdr::gain_range_t(0, 62, 2);
	//	}
	const uint32_t clip_gain =
		(gain >= 62) ? 62 : ((uint32_t) ((round((double) gain / 2))) * 2);
	const uint8_t value = (uint8_t) clip_gain;
	uart_printf("set rx bb gain: %d -> %d\n", gain, value);
	if (RADIO_OK !=
	    radio_set_gain(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_GAIN_RX_VGA,
		    (radio_gain_t) {.db = value})) {
		uart_printf("standalone RX setup failed: BB Gain\n");
	}
}

void tx_set_rf_gain(const uint32_t gain)
{
	//  if ( "RF" == name ) {
	// 		return osmosdr::gain_range_t( 0, 14, 14 );
	// 	}
	const uint8_t value = (gain >= 14) ? 1 : 0;
	uart_printf("set tx rf gain: %d -> %d\n", gain, value);
	if (RADIO_OK !=
	    radio_set_gain(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_GAIN_RF_AMP,
		    (radio_gain_t) {.db = value})) {
		uart_printf("standalone TX setup failed: RF Gain\n");
	}
}

void tx_set_if_gain(const double gain)
{
	// if ( "IF" == name ) {
	// 	return osmosdr::gain_range_t( 0, 47, 1 );
	// }
	const uint8_t value = (gain >= 47) ? 47 : gain;
	uart_printf("set tx if gain: %d -> %d\n", gain, value);
	if (radio_set_gain(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_GAIN_TX_VGA,
		    (radio_gain_t) {.db = value})) {
		hackrf_ui()->set_bb_tx_vga_gain(value);
	}
}

// FROM HACKRF HOST CODE
typedef struct {
	uint32_t bandwidth_hz;
} max2837_ft_t;

static const max2837_ft_t max2837_ft[] = {
	{1750000},
	{2500000},
	{3500000},
	{5000000},
	{5500000},
	{6000000},
	{7000000},
	{8000000},
	{9000000},
	{10000000},
	{12000000},
	{14000000},
	{15000000},
	{20000000},
	{24000000},
	{28000000},
	{0}};

uint32_t _hackrf_compute_baseband_filter_bw(const uint32_t bandwidth_hz)
{
	const max2837_ft_t* p = max2837_ft;
	while (p->bandwidth_hz != 0) {
		if (p->bandwidth_hz >= bandwidth_hz) {
			break;
		}
		p++;
	}

	/* Round down (if no equal to first entry) and if > bandwidth_hz */
	if (p != max2837_ft) {
		if (p->bandwidth_hz > bandwidth_hz)
			p--;
	}

	return p->bandwidth_hz;
}

uint32_t _bandwidth_clip(const double bandwidth)
{
	uint32_t baseband_filter_bw[] = {
		1750000,
		2500000,
		3500000,
		5000000,
		5500000,
		6000000,
		7000000,
		8000000,
		9000000,
		10000000,
		12000000,
		14000000,
		15000000,
		20000000,
		24000000,
		28000000};

	uint32_t closest = baseband_filter_bw[0];
	uint32_t min_diff = abs((int) bandwidth - (int) closest);

	for (int i = 1; i < 16; i++) {
		uint32_t diff = abs((int) bandwidth - (int) baseband_filter_bw[i]);
		if (diff < min_diff) {
			min_diff = diff;
			closest = baseband_filter_bw[i];
		}
	}

	uart_printf("baseband clip: %f -> %u\n", bandwidth, closest);
	return closest;
}

void set_baseband_filter_bandwidth(const double bandwidth)
{
	// compute best default value depending on sample rate (auto filter)
	const uint32_t bw = _hackrf_compute_baseband_filter_bw(_bandwidth_clip(bandwidth));
	// NOTE:
	// In the host: hackrf_set_baseband_filter_bandwidth
	// 	result = libusb_control_transfer(
	//		device->usb_device,
	//		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
	//			LIBUSB_RECIPIENT_DEVICE,
	//		HACKRF_VENDOR_REQUEST_BASEBAND_FILTER_BANDWIDTH_SET,
	//		bandwidth_hz & 0xffff,
	//		bandwidth_hz >> 16,
	//		NULL,
	//		0,
	//		0);
	// In the Firmware: usb_vendor_request_set_baseband_filter_bandwidth
	// 		const uint32_t bandwidth = (endpoint->setup.index << 16) | endpoint->setup.value;
	uart_printf("set baseband filter bandwidth (%f) -> bw = %d\n", bandwidth, bw);
	if (RADIO_OK !=
	    radio_set_filter(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_FILTER_BASEBAND,
		    (radio_filter_t) {.hz = bandwidth})) {
		uart_printf("standalone RX setup failed: baseband filter\n");
	}
}

// FROM HACKRF HOST CODE
int set_sample_rate(const double freq)
{
	const int MAX_N = 32;
	uint32_t freq_hz, divider;
	double freq_frac = 1.0 + freq - (int) freq;
	uint64_t a, m;
	int i, e;

	union {
		uint64_t u64;
		double d;
	} v;

	v.d = freq;

	e = (v.u64 >> 52) - 1023;

	m = ((1ULL << 52) - 1);

	v.d = freq_frac;
	v.u64 &= m;

	m &= ~((1 << (e + 4)) - 1);

	a = 0;

	for (i = 1; i < MAX_N; i++) {
		a += v.u64;
		if (!(a & m) || !(~a & m))
			break;
	}

	if (i == MAX_N)
		i = 1;

	freq_hz = (uint32_t) (freq * i + 0.5);
	divider = i;

	// uart_printf("freq_hz: %d, divider: %d\n", freq_hz, divider);
	// NOTE:
	//	 In the host: hackrf_set_sample_rate_manual(device, freq_hz, divider);
	//	result = libusb_control_transfer(
	//		device->usb_device,
	//		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR |
	//			LIBUSB_RECIPIENT_DEVICE,
	//		HACKRF_VENDOR_REQUEST_SAMPLE_RATE_SET,
	//		0,
	//		0,
	//		(unsigned char*) &set_fracrate_params,
	//		length,
	//		0);
	//	if (result < length) {
	//		last_libusb_error = result;
	//		return HACKRF_ERROR_LIBUSB;
	//	} else {
	//		return hackrf_set_baseband_filter_bandwidth(
	//			device,
	//			hackrf_compute_baseband_filter_bw(
	//				(uint32_t) (0.75 * freq_hz / divider)));
	//	}
	// In the firmware: usb_vendor_request_set_sample_rate_frac
	//		if (sample_rate_frac_set(
	//		    set_sample_r_params.freq_hz * 2,
	//		    set_sample_r_params.divider)) {
	//			usb_transfer_schedule_ack(endpoint->in);
	//			return USB_REQUEST_STATUS_OK;
	//		}
	sample_rate_frac_set(freq_hz * 2, divider);
	const uint32_t bw = _hackrf_compute_baseband_filter_bw(0.75 * freq_hz / divider);
	uart_printf(
		"set_sample_rate(%f) -> freq_hz=%u divider=%u, setting baseband filter bw to %u\n",
		freq,
		freq_hz,
		divider,
		bw);
	if (RADIO_OK !=
	    radio_set_filter(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_FILTER_BASEBAND,
		    (radio_filter_t) {.hz = bw})) {
		uart_printf(
			"standalone RX setup failed: set sample rate and baseband filter\n");
	}
}

void set_centre_frequency(const double freq)
{
#define APPLY_PPM_CORR(val, ppm) ((val) * (1.0 + (ppm) * 0.000001))
	const double _freq_corr = 0;
	const uint64_t corr_freq = (uint64_t) (APPLY_PPM_CORR(freq, _freq_corr));
	// hackrf_set_freq(freq_hz)
	uart_printf("set frequency: %llu\n", (unsigned long long) corr_freq);
	set_freq(corr_freq);
}

void set_antenna_enable(const bool enable)
{
	if (RADIO_OK !=
	    radio_set_antenna(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_ANTENNA_BIAS_TEE,
		    (radio_antenna_t) {.enable = enable})) {
		uart_printf("standalone RX setup failed: antenna enable\n");
	}
}