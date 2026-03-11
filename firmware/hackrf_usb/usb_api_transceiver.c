
/*
 * Copyright 2012-2022 Great Scott Gadgets <info@greatscottgadgets.com>
 * Copyright 2012 Jared Boone
 * Copyright 2013 Benjamin Vernoux
 *
 * This file is part of HackRF.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include "hackrf_ui.h"
#include "operacake_sctimer.h"

#include <libopencm3/cm3/vector.h>
#include <libopencm3/lpc43xx/m4/nvic.h>
#include "usb_api_m0_state.h"

#include "usb_api_cpld.h"
#include "max2839.h"
#include "rf_path.h"
#include "tuning.h"
#include "streaming.h"
#include "usb.h"
#include "usb_queue.h"
#include "platform_detect.h"

#include <stddef.h>
#include <string.h>

#include "usb_endpoint.h"
#include "usb_api_sweep.h"

#include <math.h>
#include <stdint.h>
#include "sine_table.h"

#include "max2837.h"
#include "rf_path.h"
#include "sgpio.h"
#include "usb_bulk_buffer.h"
#include "usb_api_transceiver.h"

#include <stdbool.h>
#include "gpio.h"
#include <libopencm3/lpc43xx/timer.h>
#include <stdlib.h>
#include "uart.h"

// MAPPING FROM THE GR-OSMOSDR PROJECT
void rx_set_if_gain(const uint32_t gain)
{
	//	if ( "IF" == name ) {
	//		return osmosdr::gain_range_t( 0, 40, 8 );
	//	}
	//	in-between ranges, clip to nearest
	const uint32_t clip_gain = (gain >= 40) ? 40 : ((uint32_t) (round(gain / 8))) * 8;
	const uint8_t value = (uint8_t) clip_gain;
	uart_printf("set rx if gain: %f -> %d\n", gain, value);
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
	const uint32_t clip_gain = (gain == 14) ? 14 : 0;
	const uint8_t value = (clip_gain == 14) ? 1 : 0;
	uart_printf("set rx rf gain: %f -> value=%u\n", gain, value);
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
	uart_printf("set rx bb gain: %f -> %d\n", gain, value);
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
	const uint32_t clip_gain = (gain == 14) ? 14 : 0;
	const uint8_t value = (clip_gain == 14) ? 1 : 0;
	uart_printf("set tx rf gain: %f -> %d\n", gain, value);
	if (RADIO_OK !=
	    radio_set_gain(
		    &radio,
		    RADIO_CHANNEL0,
		    RADIO_GAIN_RX_LNA,
		    (radio_gain_t) {.db = value})) {
		uart_printf("standalone RX setup failed: IF gain\n");
	}
}

void tx_set_if_gain(const double gain)
{
	// if ( "IF" == name ) {
	// 	return osmosdr::gain_range_t( 0, 47, 1 );
	// }
	const uint32_t clip_gain = (gain >= 47) ? 47 : gain;
	const uint8_t value = clip_gain;
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

void set_baseband_filter_bandwidth(double bandwidth)
{
	// compute best default value depending on sample rate (auto filter)
	const uint32_t bw = _hackrf_compute_baseband_filter_bw(bandwidth);
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

void set_centre_frequency(double freq)
{
#define APPLY_PPM_CORR(val, ppm) ((val) * (1.0 + (ppm) * 0.000001))
	const double _freq_corr = 0;
	const uint64_t corr_freq = (uint64_t) (APPLY_PPM_CORR(freq, _freq_corr));
	// hackrf_set_freq(freq_hz)
	uart_printf("set frequency: %llu\n", (unsigned long long) corr_freq);
	set_freq(corr_freq);
}

extern uint32_t __m0_start__;
extern uint32_t __m0_end__;
extern uint32_t __ram_m0_start__;
extern uint32_t _etext_ram, _text_ram, _etext_rom;
extern void tx_autostart_init(void);

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

#define USB_TRANSFER_SIZE 256
#define SINE_FREQ         200000 // 200 kHz sine wave

#define SAMPLE_RATE        10000000  // 10 Msps sample rate
#define BASEBAND_FILTER_BW 100000    // 100 kHz baseband filter bandwidth
#define CENTRE_FREQ        915000000 // 915 MHz

#define RF_GAIN 30 // 30 dB RF gain
#define IF_GAIN 40 // 40 dB IF gain
#define BB_GAIN 30 // 30 dB baseband gain

const uint32_t TX_BIT_SAMPLES = 10000000;
const uint32_t RX_BIT_SAMPLES = TX_BIT_SAMPLES;

typedef struct {
	uint32_t freq_mhz;
	uint32_t freq_hz;
} set_freq_params_t;

set_freq_params_t set_freq_params;

struct set_freq_explicit_params {
	uint64_t if_freq_hz; /* intermediate frequency */
	uint64_t lo_freq_hz; /* front-end local oscillator frequency */
	uint8_t path;        /* image rejection filter path */
};

struct set_freq_explicit_params explicit_params;

typedef struct {
	uint32_t freq_hz;
	uint32_t divider;
} set_sample_r_params_t;

set_sample_r_params_t set_sample_r_params;

usb_request_status_t usb_vendor_request_set_baseband_filter_bandwidth(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		const uint32_t bandwidth =
			(endpoint->setup.index << 16) | endpoint->setup.value;
		// uart_printf("radio_set_filter(bandwidth=%u)\n", bandwidth);
		radio_error_t result = radio_set_filter(
			&radio,
			RADIO_CHANNEL0,
			RADIO_FILTER_BASEBAND,
			(radio_filter_t) {.hz = bandwidth});
		if (result == RADIO_OK) {
			radio_filter_t real = radio_get_filter(
				&radio,
				RADIO_CHANNEL0,
				RADIO_FILTER_BASEBAND);
			uart_printf("set_filter_bw(%u) real=%u\n", bandwidth, real.hz);
			hackrf_ui()->set_filter_bw(real.hz);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_freq(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		usb_transfer_schedule_block(
			endpoint->out,
			&set_freq_params,
			sizeof(set_freq_params_t),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		uart_printf(
			"usb_vendor_request_set_freq DATA freq_mhz=%u freq_hz=%u\n",
			set_freq_params.freq_mhz,
			set_freq_params.freq_hz);
		const uint64_t freq =
			set_freq_params.freq_mhz * 1000000ULL + set_freq_params.freq_hz;
		radio_error_t result = radio_set_frequency(
			&radio,
			RADIO_CHANNEL0,
			RADIO_FREQUENCY_RF,
			(radio_frequency_t) {.hz = freq});
		if (result == RADIO_OK) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_freq_explicit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uart_printf(
			"usb_vendor_request_set_freq_explicit SETUP if_freq_hz=%llu lo_freq_hz=%llu path=%u\n",
			explicit_params.if_freq_hz,
			explicit_params.lo_freq_hz,
			explicit_params.path);
		usb_transfer_schedule_block(
			endpoint->out,
			&explicit_params,
			sizeof(struct set_freq_explicit_params),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		uart_printf(
			"usb_vendor_request_set_freq_explicit DATA if_freq_hz=%llu lo_freq_hz=%llu path=%u\n",
			explicit_params.if_freq_hz,
			explicit_params.lo_freq_hz,
			explicit_params.path);
		radio_error_t result = radio_set_frequency(
			&radio,
			RADIO_CHANNEL0,
			RADIO_FREQUENCY_RF,
			(radio_frequency_t) {.if_hz = explicit_params.if_freq_hz,
					     .lo_hz = explicit_params.lo_freq_hz,
					     .path = explicit_params.path});
		if (result == RADIO_OK) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_sample_rate_frac(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uart_printf(
			"usb_vendor_request_set_sample_rate_frac SETUP freq_hz=%u divider=%u\n",
			set_sample_r_params.freq_hz,
			set_sample_r_params.divider);
		usb_transfer_schedule_block(
			endpoint->out,
			&set_sample_r_params,
			sizeof(set_sample_r_params_t),
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		radio_error_t result = radio_set_sample_rate(
			&radio,
			RADIO_CHANNEL0,
			RADIO_SAMPLE_RATE_CLOCKGEN,
			(radio_sample_rate_t) {
				.num = set_sample_r_params.freq_hz * 2,
				.div = set_sample_r_params.divider,
			});
		if (result == RADIO_OK) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_amp_enable(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	radio_gain_t off = {.enable = false};
	radio_gain_t on = {.enable = true};

	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uart_printf(
			"usb_vendor_request_set_amp_enable SETUP value=%u\n",
			endpoint->setup.value);
		switch (endpoint->setup.value) {
		case 0:
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_RF_AMP, off);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_RF_AMP, on);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_lna_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		radio_gain_t gain = {.db = endpoint->setup.index};
		uint8_t value =
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_RX_LNA, gain);
		endpoint->buffer[0] = value;
		uart_printf(
			"radio_set_gain(RADIO_GAIN_RX_LNA, %u) -> value=%u\n",
			gain.db,
			value);
		if (value) {
			uart_printf(
				"set_lna_gain(%u) radio_set_gain(value=%u)\n",
				endpoint->setup.index,
				value);
			hackrf_ui()->set_bb_lna_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_vga_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		radio_gain_t gain = {.db = endpoint->setup.index};
		uint8_t value =
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_RX_VGA, gain);
		endpoint->buffer[0] = value;
		uart_printf(
			"radio_set_gain(RADIO_GAIN_RX_VGA, %u) -> value=%u\n",
			gain.db,
			value);
		if (value) {
			uart_printf(
				"set_vga_gain(%u) radio_set_gain(value=%u)\n",
				endpoint->setup.index,
				value);
			hackrf_ui()->set_bb_vga_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_txvga_gain(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		radio_gain_t gain = {.db = endpoint->setup.index};
		uint8_t value =
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_TX_VGA, gain);
		endpoint->buffer[0] = value;
		uart_printf(
			"radio_set_gain(RADIO_GAIN_TX_VGA, %u) -> value=%u\n",
			gain.db,
			value);
		if (value) {
			uart_printf(
				"set_txvga_gain(%u) radio_set_gain(value=%u)\n",
				endpoint->setup.index,
				value);
			hackrf_ui()->set_bb_tx_vga_gain(endpoint->setup.index);
		}
		usb_transfer_schedule_block(
			endpoint->in,
			&endpoint->buffer,
			1,
			NULL,
			NULL);
		usb_transfer_schedule_ack(endpoint->out);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_antenna_enable(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	radio_antenna_t off = {.enable = false};
	radio_antenna_t on = {.enable = true};

	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uart_printf(
			"usb_vendor_request_set_antenna_enable SETUP value=%u\n",
			endpoint->setup.value);
		switch (endpoint->setup.value) {
		case 0:
			radio_set_antenna(
				&radio,
				RADIO_CHANNEL0,
				RADIO_ANTENNA_BIAS_TEE,
				off);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			radio_set_antenna(
				&radio,
				RADIO_CHANNEL0,
				RADIO_ANTENNA_BIAS_TEE,
				on);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

static volatile uint32_t _tx_underrun_limit;
static volatile uint32_t _rx_overrun_limit;

volatile transceiver_request_t transceiver_request = {
	.mode = TRANSCEIVER_MODE_OFF,
	.seq = 0,
};

// Must be called from an atomic context (normally USB ISR)
void request_transceiver_mode(transceiver_mode_t mode)
{
	usb_endpoint_flush(&usb_endpoint_bulk_in);
	usb_endpoint_flush(&usb_endpoint_bulk_out);

	transceiver_request.mode = mode;
	transceiver_request.seq++;
}

void fill_sine_buffer(uint8_t* buffer, uint32_t len, uint32_t* _unused_phase)
{
	const uint32_t BLOCK_SAMPLES = 8192;              // samples per half-cycle
	const uint32_t CYCLE_SAMPLES = BLOCK_SAMPLES * 2; // full sine+zero cycle
	const uint32_t BYTES_PER_SAMPLE = 2;              // I + Q
	const uint8_t MID_SCALE = 0;                      // "zero" for unsigned I/Q128

	// phase increment to step through your sine table at the right rate:
	const uint32_t phase_increment = (SINE_FREQ * SINE_TABLE_SIZE) / SAMPLE_RATE;

	// static so they survive across back-to-back fill_sine_buffer() calls:
	static uint32_t super_phase = 0; // 0…CYCLE_SAMPLES−1
	static uint32_t table_phase = 0; // 0…SINE_TABLE_SIZE−1

	uint32_t total_samples = len / BYTES_PER_SAMPLE;

	for (uint32_t s = 0; s < total_samples; s++) {
		uint32_t i = s * BYTES_PER_SAMPLE;

		if (super_phase < BLOCK_SAMPLES) {
			// --- sine part ---
			uint32_t idx = table_phase;
			buffer[i] = sine_table[2 * idx];         // I
			buffer[i + 1] = sine_table[2 * idx + 1]; // Q

			// advance through sine table
			table_phase = (table_phase + phase_increment) % SINE_TABLE_SIZE;
		} else {
			// --- zero part ---
			buffer[i] = MID_SCALE;
			buffer[i + 1] = MID_SCALE;
		}

		// advance through the 8192+8192 super-cycle
		super_phase++;
		if (super_phase >= CYCLE_SAMPLES) {
			super_phase = 0;
		}
	}
}

// Add new variables for dynamic bitstream
static uint8_t* dynamic_bit_pattern = NULL;
static size_t dynamic_pattern_len = 0;
static bool pattern_sent_once = false; // New flag to track if pattern was sent once

// Add new USB request handler for setting bitstream pattern
usb_request_status_t usb_vendor_request_set_bitstream_pattern(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		// Free previous pattern if it exists
		if (dynamic_bit_pattern) {
			free(dynamic_bit_pattern);
			dynamic_bit_pattern = NULL;
		}

		// Allocate new pattern buffer
		dynamic_pattern_len = endpoint->setup.length;
		dynamic_bit_pattern = malloc(dynamic_pattern_len);
		if (!dynamic_bit_pattern) {
			return USB_REQUEST_STATUS_STALL;
		}

		// Clear the buffer before receiving new data
		memset(dynamic_bit_pattern, 0, dynamic_pattern_len);

		// Reset the pattern sent flag for new pattern
		pattern_sent_once = false;

		// Schedule data transfer
		usb_transfer_schedule_block(
			endpoint->out,
			dynamic_bit_pattern,
			dynamic_pattern_len,
			NULL,
			NULL);
		return USB_REQUEST_STATUS_OK;
	} else if (stage == USB_TRANSFER_STAGE_DATA) {
		// Verify we received the expected amount of data
		if (endpoint->setup.length != dynamic_pattern_len) {
			free(dynamic_bit_pattern);
			dynamic_bit_pattern = NULL;
			dynamic_pattern_len = 0;
			return USB_REQUEST_STATUS_STALL;
		}

		// Clear the USB bulk buffer to ensure new pattern is used
		memset(usb_bulk_buffer, 0, USB_BULK_BUFFER_SIZE);

		// Reset the phase and counters to ensure clean start with new pattern
		static uint32_t phase = 0;
		phase = 0;

		usb_transfer_schedule_ack(endpoint->in);
		return USB_REQUEST_STATUS_OK;
	}
	return USB_REQUEST_STATUS_OK;
}

// void fill_data_buffer(uint8_t* buffer, uint32_t len, uint32_t* _unused_phase)
// {
// 	const uint32_t BYTES_PER_SAMPLE = 2; // I + Q
// 	const uint8_t MID_SCALE = 0;         // unsigned zero level

// 	// phase increment through the sine table
// 	const uint32_t phase_increment = (SINE_FREQ * SINE_TABLE_SIZE) / SAMPLE_RATE;

// 	// persistent state
// 	static uint32_t table_phase = 0;   // index into sine_table
// 	static uint32_t sample_in_bit = 0; // 0…BIT_SAMPLES−1
// 	static uint32_t bit_index = 0;     // 0…dynamic_pattern_len−1
// 	// static const uint8_t dynamic_bit_pattern[] = {1, 1, 1, 0, 1, 0, 1, 0};
// 	static const uint8_t dynamic_bit_pattern[] =
// 		{1, 0, 1, 0, 1, 0, 1, 0}; // Example pattern
// 	const uint32_t dynamic_pattern_len =
// 		sizeof(dynamic_bit_pattern) / sizeof(*dynamic_bit_pattern);

// 	const uint32_t total_samples = len / BYTES_PER_SAMPLE;

// 	for (uint32_t s = 0; s < total_samples; s++) {
// 		const uint32_t i = s * BYTES_PER_SAMPLE;
// 		if (dynamic_bit_pattern[bit_index]) {
// 			// — "1": sine output
// 			const uint32_t ti = table_phase % SINE_TABLE_SIZE;
// 			buffer[i] = sine_table[2 * ti];
// 			buffer[i + 1] = sine_table[2 * ti + 1];
// 			// uart_printf("1");
// 		} else {
// 			// — "0": flat mid-scale
// 			buffer[i] = MID_SCALE;
// 			buffer[i + 1] = MID_SCALE;
// 			// uart_printf("0");
// 		}
// 		// advance phase even when outputting "0" to maintain correct timing when we return to "1"
// 		table_phase = (table_phase + phase_increment) % SINE_TABLE_SIZE;

// 		// advance sample count; after TX_BIT_SAMPLES, move to next bit
// 		if (++sample_in_bit >= TX_BIT_SAMPLES) {
// 			sample_in_bit = 0;
// 			bit_index = (bit_index + 1) % dynamic_pattern_len;
// 			uart_printf("%d", dynamic_bit_pattern[bit_index]);
// 			// If we've completed one full pattern, set the flag
// 			if (bit_index == 0) {
// 				pattern_sent_once = true;
// 			}
// 		}
// 	}
// }

void fill_data_buffer(uint8_t* buffer, uint32_t len, uint32_t* _unused_phase)
{
	const uint32_t BYTES_PER_SAMPLE = 2; // I + Q
	const uint8_t MID_SCALE = 0;         // unsigned zero level

	// phase increment through the sine table
	const uint32_t phase_increment = (SINE_FREQ * SINE_TABLE_SIZE) / SAMPLE_RATE;

	// persistent state
	static uint32_t table_phase = 0;   // index into sine_table
	static uint32_t sample_in_bit = 0; // 0…BIT_SAMPLES−1
	static uint32_t bit_index = 0;     // 0…dynamic_pattern_len−1
	static const uint8_t dynamic_bit_pattern[] = {1, 1, 1, 0, 1, 0, 1, 0};
#define dynamic_pattern_len (sizeof(dynamic_bit_pattern) / sizeof(*dynamic_bit_pattern))

	uint32_t total_samples = len / BYTES_PER_SAMPLE;

	for (uint32_t s = 0; s < total_samples; s++) {
		uint32_t i = s * BYTES_PER_SAMPLE;
		if (dynamic_bit_pattern[bit_index]) {
			// — "1": sine output
			uint32_t ti = table_phase % SINE_TABLE_SIZE;
			buffer[i] = sine_table[2 * ti];
			buffer[i + 1] = sine_table[2 * ti + 1];
			// uart_printf("%d", buffer[i]);
		} else {
			// — "0": flat mid-scale
			buffer[i] = MID_SCALE;
			buffer[i + 1] = MID_SCALE;
			// uart_printf("%d", buffer[i]);
		}

		table_phase = (table_phase + phase_increment) % SINE_TABLE_SIZE;

		// advance sample count; after BIT_SAMPLES, move to next bit
		if (++sample_in_bit >= TX_BIT_SAMPLES) {
			sample_in_bit = 0;
			bit_index = (bit_index + 1) % dynamic_pattern_len;

			// If we've completed one full pattern, set the flag
			if (bit_index == 0) {
				pattern_sent_once = true;
			}
		}
	}
}

void fill_square_buffer(uint8_t* buffer, uint32_t len, uint32_t* phase)
{
	const uint32_t points_per_state = 8192;
	uint8_t high_val = 127;
	uint8_t low_val = 0;

	for (uint32_t i = 0; i < len; i += 2) {
		uint32_t current_point = (*phase) % (points_per_state * 2);

		if (current_point < points_per_state) {
			// High state (127 amplitude)
			buffer[i] = high_val;     // I sample
			buffer[i + 1] = high_val; // Q sample
		} else {
			// Low state (0 amplitude)
			buffer[i] = low_val;     // I sample
			buffer[i + 1] = low_val; // Q sample
		}

		// Increment phase to move to next point
		*phase = (*phase + 1) % (points_per_state * 2);
	}
}

void transceiver_shutdown(void)
{
	baseband_streaming_disable(&sgpio_config);
	operacake_sctimer_reset_state();

	usb_endpoint_flush(&usb_endpoint_bulk_in);
	usb_endpoint_flush(&usb_endpoint_bulk_out);

	led_off(LED2);
	led_off(LED3);
	radio_switch_mode(&radio, RADIO_CHANNEL0, TRANSCEIVER_MODE_OFF);
	m0_set_mode(M0_MODE_IDLE);
}

void transceiver_startup(const transceiver_mode_t mode)
{
	radio_switch_mode(&radio, RADIO_CHANNEL0, mode);
	hackrf_ui()->set_transceiver_mode(mode);

	switch (mode) {
	case TRANSCEIVER_MODE_RX_SWEEP:
	case TRANSCEIVER_MODE_RX:
		led_off(LED3);
		led_on(LED2);
		m0_set_mode(M0_MODE_RX);
		m0_state.shortfall_limit = _rx_overrun_limit;
		break;
	case TRANSCEIVER_MODE_TX:
		led_off(LED2);
		led_on(LED3);
		m0_set_mode(M0_MODE_TX_START);
		m0_state.shortfall_limit = _tx_underrun_limit;
		break;
	default:
		break;
	}

	activate_best_clock_source();
	trigger_enable(radio_get_trigger_enable(&radio, RADIO_CHANNEL0));
}

usb_request_status_t usb_vendor_request_set_transceiver_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uart_printf(
			"usb_vendor_request_set_transceiver_mode SETUP value=%u\n",
			endpoint->setup.value);
		switch (endpoint->setup.value) {
		case TRANSCEIVER_MODE_OFF:
		case TRANSCEIVER_MODE_RX:
		case TRANSCEIVER_MODE_TX:
		case TRANSCEIVER_MODE_RX_SWEEP:
		case TRANSCEIVER_MODE_CPLD_UPDATE:
			request_transceiver_mode(endpoint->setup.value);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_hw_sync_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		radio_error_t result = radio_set_trigger_enable(
			&radio,
			RADIO_CHANNEL0,
			endpoint->setup.value != 0);
		if (result == RADIO_OK) {
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		}
		return USB_REQUEST_STATUS_STALL;
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

usb_request_status_t usb_vendor_request_set_tx_underrun_limit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint32_t value = (endpoint->setup.index << 16) + endpoint->setup.value;
		_tx_underrun_limit = value;
		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_set_rx_overrun_limit(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uint32_t value = (endpoint->setup.index << 16) + endpoint->setup.value;
		_rx_overrun_limit = value;
		usb_transfer_schedule_ack(endpoint->in);
	}
	return USB_REQUEST_STATUS_OK;
}

void transceiver_bulk_transfer_complete(void* user_data, unsigned int bytes_transferred)
{
	(void) user_data;
	m0_state.m4_count += bytes_transferred;
}

#define BIT_PACKET_SIZE 4

static void rx_standalone_autostart(void)
{
	set_sample_rate(SAMPLE_RATE);
	set_baseband_filter_bandwidth(BASEBAND_FILTER_BW);
	set_centre_frequency(CENTRE_FREQ);
	rx_set_rf_gain(RF_GAIN);
	rx_set_if_gain(IF_GAIN);
	rx_set_bb_gain(BB_GAIN);

	// if (RADIO_OK !=
	//     radio_set_antenna(
	// 	    &radio,
	// 	    RADIO_CHANNEL0,
	// 	    RADIO_ANTENNA_BIAS_TEE,
	// 	    (radio_antenna_t) {.enable = false})) {
	// 	uart_printf("standalone RX setup failed: antenna");
	// 	return;
	// }

	// if (RADIO_OK != radio_set_trigger_enable(&radio, RADIO_CHANNEL0, false)) {
	// 	uart_printf("standalone RX setup failed: trigger");
	// 	return;
	// }

	request_transceiver_mode(TRANSCEIVER_MODE_RX);
	uart_printf("standalone RX autostart requested\n");
}

void rx_mode(uint32_t seq)
{
	rx_standalone_autostart();
	uint32_t usb_count = 0;
	transceiver_startup(TRANSCEIVER_MODE_RX);
	baseband_streaming_enable(&sgpio_config);

	// Constants matching the transmitter
	uint32_t sample_count = 0;
	uint64_t mag2_sum = 0; // Sum of magnitude squared
	bool current_bit = false;

	uint32_t noise_floor = 0;
	const uint32_t THRESHOLD_MARGIN = 500;

	while (1) {
		uint8_t* buffer = &usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK];
		for (uint32_t i = 0; i < USB_TRANSFER_SIZE; i += 2) {
			int32_t I = (int8_t) buffer[i];
			int32_t Q = (int8_t) buffer[i + 1];
			// consume sample to avoid overflow
			usb_count += 2;
			m0_state.m4_count += 2;

			uint32_t mag2 = I * I + Q * Q;
			mag2_sum += mag2;
			sample_count++;

			if (sample_count >= RX_BIT_SAMPLES) {
				uint32_t mag2_avg = mag2_sum / RX_BIT_SAMPLES;

				// Update noise floor (slow moving average)
				noise_floor = (noise_floor * 15 + mag2_avg) / 16;

				// Adaptive threshold
				uint32_t adaptive_threshold =
					noise_floor + THRESHOLD_MARGIN;

				current_bit = (mag2_avg > adaptive_threshold);

				uart_printf("%d", current_bit);

				sample_count = 0;
				mag2_sum = 0;
			}
		}
	}

	transceiver_shutdown();
}

void tx_mode(uint32_t seq)
{
	uart_printf("tx_mode");
	unsigned int usb_count = 0;
	bool started = false;

	transceiver_startup(TRANSCEIVER_MODE_TX);

	// Set up OUT transfer of buffer 0.
	usb_transfer_schedule_block(
		&usb_endpoint_bulk_out,
		&usb_bulk_buffer[0x0000],
		USB_TRANSFER_SIZE,
		transceiver_bulk_transfer_complete,
		NULL);
	usb_count += USB_TRANSFER_SIZE;

	while (transceiver_request.seq == seq) {
		if (!started && (m0_state.m4_count == USB_BULK_BUFFER_SIZE)) {
			// Buffer is now full, start streaming.
			baseband_streaming_enable(&sgpio_config);
			started = true;
		}
		if ((usb_count - m0_state.m0_count) <= USB_TRANSFER_SIZE) {
			usb_transfer_schedule_block(
				&usb_endpoint_bulk_out,
				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK],
				USB_TRANSFER_SIZE,
				transceiver_bulk_transfer_complete,
				NULL);
			usb_count += USB_TRANSFER_SIZE;
		}
	}

	transceiver_shutdown();
}

// void tx_mode(uint32_t seq)
// {
// 	uart_printf("TX mode");
// 	init_sine_table();
// #if false
// 	set_sample_rate(SAMPLE_RATE);
// 	set_baseband_filter_bandwidth(15000000);
// 	set_centre_frequency(CENTRE_FREQ);
// 	// set_rf_gain(RF_GAIN);
// 	tx_set_if_gain(40);
// 	// set_bb_gain(BB_GAIN);
// 	// radio_set_antenna(
// 	// 	&radio,
// 	// 	RADIO_CHANNEL0,
// 	// 	RADIO_ANTENNA_BIAS_TEE,
// 	// 	(radio_antenna_t) {.enable = true});
// 	// if (RADIO_OK != radio_set_trigger_enable(&radio, RADIO_CHANNEL0, false)) {
// 	// 	uart_printf("standalone TX setup failed: trigger");
// 	// 	return;
// 	// }
// #else
// 	sample_rate_frac_set(SAMPLE_RATE, 1);         // 20 MS/s
// 	// baseband_filter_bandwidth_set(15000000);   // 15 MHz bandwidth
// 	set_freq(CENTRE_FREQ);                       // Frequency 915 MHz
// 	max283x_set_txvga_gain(&max283x, 47);      // Maximum TX gain
// 	rf_path_set_lna(&rf_path, 1);              // Enable LNA
// 	rf_path_set_antenna(&rf_path, 1);          // Select antenna path
// #endif
// 	request_transceiver_mode(TRANSCEIVER_MODE_TX);

// 	// Start transceiver once
// 	transceiver_startup(TRANSCEIVER_MODE_TX);
// 	baseband_streaming_enable(&sgpio_config);

// 	// Phase tracking for waveform continuity
// 	static uint32_t phase = 0;

// 	// USB bulk transfer count initialization
// 	uint32_t usb_count = 0;

// 	// Continuously fill the USB bulk buffer with IQ data
// 	while (1) {
// 		// Wait until there's space to safely write
// 		if (m0_state.m0_count == 0) {
// 			uart_printf("Stalled\n");
// 		}
// 		if ((usb_count - m0_state.m0_count) <=
// 		    USB_BULK_BUFFER_SIZE - USB_TRANSFER_SIZE) {
// 			// Fill the buffer with your desired waveform
// 			fill_data_buffer(
// 				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK],
// 				USB_TRANSFER_SIZE,
// 				&phase);

// 			// Atomically update shared counters
// 			nvic_disable_irq(NVIC_M0CORE_IRQ);
// 			m0_state.m4_count += USB_TRANSFER_SIZE;
// 			m0_state.m0_count += USB_TRANSFER_SIZE;
// 			usb_count += USB_TRANSFER_SIZE;
// 			nvic_enable_irq(NVIC_M0CORE_IRQ);
// 		}

// 		// Optional small delay
// 		__asm__("nop");
// 	}

// 	// Never reaches here, but cleanup code can be included
// 	transceiver_shutdown();
// }

void off_mode(uint32_t seq)
{
	hackrf_ui()->set_transceiver_mode(TRANSCEIVER_MODE_OFF);

	while (transceiver_request.seq == seq) {}
}