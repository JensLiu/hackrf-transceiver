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

#include "usb_api_transceiver.h"

#include "hackrf_ui.h"
#include "operacake_sctimer.h"

#include <libopencm3/cm3/vector.h>
#include "usb_bulk_buffer.h"
#include "usb_api_m0_state.h"

#include "usb_api_cpld.h" // Remove when CPLD update is handled elsewhere

#include "max2837.h"
#include "max2839.h"
#include "rf_path.h"
#include "tuning.h"
#include "streaming.h"
#include "usb.h"
#include "usb_queue.h"
#include "platform_detect.h"

#include <stddef.h>
#include <string.h>
#include "uart.h"

#include "usb_endpoint.h"

#include "usb_api_sweep.h"
#include <math.h>

// CUSTOM IMPORTS
#include "custom_config.h"
#include "sine_table.h"
#include "gr_lib.h"

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
		uart_printf(
			"usb_vendor_request_set_baseband_filter_bandwidth SETUP index=%u value=%u\n",
			endpoint->setup.index,
			endpoint->setup.value);
		const uint32_t bandwidth =
			(endpoint->setup.index << 16) | endpoint->setup.value;
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
		uart_printf(
			"usb_vendor_request_set_freq SETUP freq_mhz=%u freq_hz=%u\n",
			set_freq_params.freq_mhz,
			set_freq_params.freq_hz);
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
		uart_printf(
			"usb_vendor_request_set_lna_gain SETUP index=%u\n",
			endpoint->setup.index);
		radio_gain_t gain = {.db = endpoint->setup.index};
		uint8_t value =
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_RX_LNA, gain);
		endpoint->buffer[0] = value;
		if (value) {
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
		uart_printf(
			"usb_vendor_request_set_vga_gain SETUP index=%u\n",
			endpoint->setup.index);
		radio_gain_t gain = {.db = endpoint->setup.index};
		uint8_t value =
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_RX_VGA, gain);
		endpoint->buffer[0] = value;
		if (value) {
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
		uart_printf(
			"usb_vendor_request_set_txvga_gain SETUP index=%u\n",
			endpoint->setup.index);
		radio_gain_t gain = {.db = endpoint->setup.index};
		uint8_t value =
			radio_set_gain(&radio, RADIO_CHANNEL0, RADIO_GAIN_TX_VGA, gain);
		endpoint->buffer[0] = value;
		if (value) {
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

// Runtime TX pattern (host update path can be wired in later).
static uint8_t tx_bit_pattern[TX_PATTERN_MAX_BITS];
static uint32_t tx_bit_pattern_len = 0;
static uint32_t tx_bit_pattern_index = 0;

static void tx_set_bit_pattern(const uint8_t* bits, uint32_t len)
{
	if ((bits == NULL) || (len == 0)) {
		return;
	}

	if (len > TX_PATTERN_MAX_BITS) {
		len = TX_PATTERN_MAX_BITS;
	}

	for (uint32_t i = 0; i < len; i++) {
		tx_bit_pattern[i] = bits[i] ? 1 : 0;
	}

	tx_bit_pattern_len = len;
	tx_bit_pattern_index = 0;
}

static void tx_set_default_pattern(void)
{
	static const uint8_t default_pattern[] = CUSTOM_BIT_PATTERN;
	tx_set_bit_pattern(default_pattern, sizeof(default_pattern));
}

void fill_data_buffer(uint8_t* buffer, uint32_t len, uint32_t* _unused_phase)
{
	const uint32_t BYTES_PER_SAMPLE = 2; // I + Q
	const uint8_t MID_SCALE = 0;         // unsigned zero level
	const uint32_t SINE_TABLE_MASK = SINE_TABLE_SIZE - 1;

	// phase increment through the sine table
	const uint32_t phase_increment = (SINE_FREQ * SINE_TABLE_SIZE) / TX_SAMPLE_RATE;

	// persistent state
	static uint32_t table_phase = 0;   // index into sine_table
	static uint32_t sample_in_bit = 0; // 0…BIT_SAMPLES−1

	// if (tx_bit_pattern_len == 0) {
	// 	memset(buffer, 0, len);
	// 	return;
	// }

	uint32_t total_samples = len / BYTES_PER_SAMPLE;
	// if (len % BYTES_PER_SAMPLE != 0) {
	// 	// Handle the case where len is not a multiple of BYTES_PER_SAMPLE
	// 	uart_printf(
	// 		"Warning: buffer length %u is not a multiple of bytes per sample %u\n",
	// 		len,
	// 		BYTES_PER_SAMPLE);
	// }

	for (uint32_t s = 0; s < total_samples; s++) {
		uint32_t i = s * BYTES_PER_SAMPLE;
		if (tx_bit_pattern[tx_bit_pattern_index]) {
			// — "1": sine output
			buffer[i] = sine_table[2 * table_phase];
			buffer[i + 1] = sine_table[2 * table_phase + 1];
		} else {
			// — "0": flat mid-scale
			buffer[i] = MID_SCALE;
			buffer[i + 1] = MID_SCALE;
		}
		table_phase = (table_phase + phase_increment) & SINE_TABLE_MASK;

		// advance sample count; after BIT_SAMPLES, move to next bit
		if (++sample_in_bit >= TX_BIT_SAMPLES) {
			sample_in_bit = 0;
			tx_bit_pattern_index =
				(tx_bit_pattern_index + 1) % tx_bit_pattern_len;
		}
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

float _rx_lut[0xff];

void init_rx_lookup_table(void)
{
	for (unsigned int i = 0; i <= 0xff; i++) {
		_rx_lut[i] = ((float) ((int8_t) i)) * (1.0f / 128.0f);
	}
	uart_printf("RX Lookup Table initialised\n");
}

void rx_signal_process(uint32_t usb_count)
{
#define N_BITS_PER_UNIT   8
#define N_SAMPLES_PER_BIT 1000
#define GLITCH_TOLERANCE  50
	static uint32_t sample_count = 0;
	static bool clean_bit = 0; // currently accepted
	static uint32_t glitch_count = 0;
	static bool bit_buffer[1024];
	static uint32_t bit_buffer_index = 0;

	for (uint32_t i = 0; i < USB_TRANSFER_SIZE; i += 2) {
		const uint8_t* const buffer =
			&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK];
		const float I = _rx_lut[buffer[i]];
		const float Q = _rx_lut[buffer[i + 1]];
		// uart_printf("I/Q Data: I=%f Q=%f\n", I, Q);

		const float mag = sqrt(I * I + Q * Q);
		const bool raw_bit = ({ // bits received (with noise)
			// threshold logic from GNU radio
			static bool thres_last_bit = 0; // < static variable
			const float thres_low = 0.1;
			const float thres_high = 0.5;
			if (mag < thres_low) {
				thres_last_bit = 0;
			} else if (mag > thres_high) {
				thres_last_bit = 1;
			}
			thres_last_bit;
		});

		// glitch filtering (tolerate some noise but flip bit if it exceeds the threshold)
		if (raw_bit != clean_bit) {
			glitch_count++;
			if (glitch_count >= GLITCH_TOLERANCE) {
				clean_bit = raw_bit; // flip bit
				glitch_count = 0;
				// sample count realignment
				sample_count = GLITCH_TOLERANCE;
			} else {
				// uart_printf(
				// 	"Glitch: raw_bit=%d clean_bit=%d glitch_count=%u\n",
				// 	raw_bit,
				// 	clean_bit,
				// 	glitch_count);
			}
		} else {
			glitch_count = 0;
		}

		// centre sampling
		sample_count++;
		if (sample_count == N_SAMPLES_PER_BIT / 2) {
			bit_buffer[bit_buffer_index++] = clean_bit;
			if (bit_buffer_index >= N_BITS_PER_UNIT) {
				// Process a full unit of bits
				uint8_t byte = 0;
				// uart_printf("got byte: ");
				for (int i = 0; i < N_BITS_PER_UNIT; i++) {
					// uart_printf(
					// 	"bit_buffer[%d] = %d\n",
					// 	i,
					// 	bit_buffer[i]);
					byte |= (bit_buffer[i] << (7 - i));
					uart_printf("%d", bit_buffer[i]);
				}
				// uart_printf("Sending byte: 0x%02x\n", byte);
				uart_printf("\n");
				bit_buffer_index = 0;
			}
		}
		// reset
		if (sample_count >= N_SAMPLES_PER_BIT) {
			sample_count = 0;
		}
	}
}

#ifndef CUSTOM_RX_MODE
void rx_mode(uint32_t seq)
{
	uint32_t usb_count = 0;

	transceiver_startup(TRANSCEIVER_MODE_RX);

	baseband_streaming_enable(&sgpio_config);

	while (transceiver_request.seq == seq) {
		if ((m0_state.m0_count - usb_count) >= USB_TRANSFER_SIZE) {
			// BEGIN signal process
			// rx_signal_process(usb_count);
			// END   signal process
			usb_transfer_schedule_block(
				&usb_endpoint_bulk_in,
				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK],
				USB_TRANSFER_SIZE,
				transceiver_bulk_transfer_complete,
				NULL);
			usb_count += USB_TRANSFER_SIZE;
		}
	}

	transceiver_shutdown();
}
#else
static void rx_standalone_autostart(void)
{
	// set_sample_rate(SAMPLE_RATE);
	// set_baseband_filter_bandwidth(BASEBAND_FILTER_BW);
	// set_centre_frequency(CENTRE_FREQ);
	// rx_set_rf_gain(RF_GAIN);
	// rx_set_if_gain(IF_GAIN);
	// rx_set_bb_gain(BB_GAIN);

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
	#ifdef RX_SAMPLE_RATE
	set_sample_rate(RX_SAMPLE_RATE);
	#endif
	#ifdef RX_BASEBAND_FILTER_BW
	set_baseband_filter_bandwidth(RX_BASEBAND_FILTER_BW);
	#endif
	#ifdef CENTRE_FREQ
	set_centre_frequency(CENTRE_FREQ);
	#endif
	#ifdef RX_RF_GAIN
	rx_set_rf_gain(RX_RF_GAIN);
	#endif
	#ifdef RX_IF_GAIN
	rx_set_if_gain(RX_IF_GAIN);
	#endif
	#ifdef RX_ANTENNA_ENABLE
	set_antenna_enable(RX_ANTENNA_ENABLE);
	#endif

	uint32_t usb_count = 0;
	transceiver_startup(TRANSCEIVER_MODE_RX);
	baseband_streaming_enable(&sgpio_config);

	// Constants matching the transmitter
	uint32_t sample_count = 0;
	uint64_t mag2_sum = 0; // Sum of magnitude squared
	bool current_bit = false;
	uint8_t bit_buffer[USB_TRANSFER_SIZE]; // Buffer to store detected bits
	uint32_t bit_buffer_index = 0;
	uint8_t tx_buffer[USB_TRANSFER_SIZE]; // Separate buffer for USB transfers
	uint32_t noise_floor = 0;

	while (1) {
		if ((m0_state.m0_count - usb_count) >= USB_TRANSFER_SIZE) {
			uint8_t* buffer =
				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK];

			for (uint32_t i = 0; i < USB_TRANSFER_SIZE; i += 2) {
				int32_t I = (int8_t) buffer[i];
				int32_t Q = (int8_t) buffer[i + 1];

				uint32_t mag2 = I * I + Q * Q;
				// uart_printf("%d\t", mag2);
				mag2_sum += mag2;
				sample_count++;

				if (sample_count >= RX_BIT_SAMPLES) {
					uint32_t mag2_avg = mag2_sum /
						RX_BIT_SAMPLES; // TODO: use power of 2 for division efficiency

					// Update noise floor (slow moving average)
					noise_floor = (noise_floor * 15 + mag2_avg) / 16;

					// Adaptive threshold
					const uint32_t adaptive_threshold =
						noise_floor + RX_THRESHOLD_MARGIN;

					current_bit = (mag2_avg > adaptive_threshold);
					// uart_printf("mag2_avg=%u noise_floor=%u threshold=%u bit=%d\n",
					// 	mag2_avg,
					// 	noise_floor,
					// 	adaptive_threshold,
					// 	current_bit);
					// uart_printf("%d", current_bit);
					// uart_printf("%d\t", mag2_avg);
					// uart_printf("%d", current_bit);
					bit_buffer[bit_buffer_index++] =
						current_bit ? 1 : 0;

					if (bit_buffer_index >= RX_BIT_PACKET_SIZE) {
						memcpy(tx_buffer,
						       bit_buffer,
						       RX_BIT_PACKET_SIZE);

						usb_transfer_schedule_block(
							&usb_endpoint_bulk_in,
							tx_buffer,
							RX_BIT_PACKET_SIZE,
							transceiver_bulk_transfer_complete,
							NULL);

						while (!usb_endpoint_bulk_in
								.transfer_complete) {
							__asm__("nop");
						}

						bit_buffer_index = 0;
					}

					sample_count = 0;
					mag2_sum = 0;
				}
			}

			// if (m0_state.num_shortfalls > 0) {
			// 	uart_printf(
			// 		"\nWarning: M0 shortfall count = %u\n",
			// 		m0_state.num_shortfalls);
			// }

			usb_count += USB_TRANSFER_SIZE;
			m0_state.m4_count += USB_TRANSFER_SIZE;
		}
	}

	transceiver_shutdown();
}

// void rx_mode(uint32_t seq)
// {
// 	// set_sample_rate(RX_SAMPLE_RATE);
// 	// set_freq(CENTRE_FREQ);
// 	// max283x_set_lna_gain(&max283x, 40);
// 	// rf_path_set_lna(&rf_path, 1);
// 	// rf_path_set_antenna(&rf_path, 1);

// 	#ifdef RX_SAMPLE_RATE
// 	set_sample_rate(RX_SAMPLE_RATE);
// 	#endif
// 	#ifdef RX_BASEBAND_FILTER_BW
// 	set_baseband_filter_bandwidth(RX_BASEBAND_FILTER_BW);
// 	#endif
// 	#ifdef CENTRE_FREQ
// 	set_centre_frequency(CENTRE_FREQ);
// 	#endif
// 	#ifdef RX_RF_GAIN
// 	rx_set_rf_gain(RX_RF_GAIN);
// 	#endif
// 	#ifdef RX_IF_GAIN
// 	rx_set_if_gain(RX_IF_GAIN);
// 	#endif
// 	#ifdef RX_ANTENNA_ENABLE
// 	set_antenna_enable(RX_ANTENNA_ENABLE);
// 	#endif

// 	uint32_t usb_count = 0;
// 	transceiver_startup(TRANSCEIVER_MODE_RX);
// 	baseband_streaming_enable(&sgpio_config);

// 	// Constants matching the transmitter
// 	uint32_t sample_count = 0;
// 	uint64_t mag2_sum = 0; // Sum of magnitude squared
// 	bool current_bit = false;
// 	uint8_t bit_buffer[USB_TRANSFER_SIZE]; // Buffer to store detected bits
// 	uint32_t bit_buffer_index = 0;
// 	uint8_t tx_buffer[USB_TRANSFER_SIZE]; // Separate buffer for USB transfers

// 	uint32_t noise_floor = 0;
// 	const uint32_t THRESHOLD_MARGIN = 500;

// 	while (1) {
// 		if ((m0_state.m0_count - usb_count) >= USB_TRANSFER_SIZE) {
// 			uint8_t* buffer =
// 				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK];

// 			for (uint32_t i = 0; i < USB_TRANSFER_SIZE; i += 2) {
// 				int32_t I = (int8_t) buffer[i];
// 				int32_t Q = (int8_t) buffer[i + 1];

// 				uint32_t mag2 = I * I + Q * Q;
// 				mag2_sum += mag2;
// 				sample_count++;

// 				if (sample_count >= RX_BIT_SAMPLES) {
// 					uint32_t mag2_avg = mag2_sum / RX_BIT_SAMPLES;

// 					// Update noise floor (slow moving average)
// 					noise_floor = (noise_floor * 15 + mag2_avg) / 16;

// 					// Adaptive threshold
// 					uint32_t adaptive_threshold =
// 						noise_floor + THRESHOLD_MARGIN;

// 					current_bit = (mag2_avg > adaptive_threshold);

// 					bit_buffer[bit_buffer_index++] =
// 						current_bit ? 1 : 0;

// 					if (bit_buffer_index >= RX_BIT_PACKET_SIZE) {
// 						memcpy(tx_buffer,
// 						       bit_buffer,
// 						       RX_BIT_PACKET_SIZE);

// 						usb_transfer_schedule_block(
// 							&usb_endpoint_bulk_in,
// 							tx_buffer,
// 							RX_BIT_PACKET_SIZE,
// 							transceiver_bulk_transfer_complete,
// 							NULL);

// 						while (!usb_endpoint_bulk_in
// 								.transfer_complete) {
// 							__asm__("nop");
// 						}

// 						bit_buffer_index = 0;
// 					}

// 					if (current_bit) {
// 						led_on(LED3);
// 					} else {
// 						led_off(LED3);
// 					}

// 					sample_count = 0;
// 					mag2_sum = 0;
// 				}
// 			}

// 			usb_count += USB_TRANSFER_SIZE;
// 			m0_state.m4_count += USB_TRANSFER_SIZE;
// 			// m0_state.m0_count += USB_TRANSFER_SIZE;
// 		}
// 	}

// 	transceiver_shutdown();
// }
#endif

#ifndef CUSTOM_TX_MODE
void tx_mode(uint32_t seq)
{
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
#else
void tx_mode(uint32_t seq)
{
	uart_printf("tx mode");
	init_sine_table();
	if (tx_bit_pattern_len == 0) {
		tx_set_default_pattern();
	}

	// Radio configuration
	#ifdef TX_SAMPLE_RATE
	set_sample_rate(TX_SAMPLE_RATE);
	#endif
	#ifdef TX_BASEBAND_FILTER_BW
	set_baseband_filter_bandwidth(TX_BASEBAND_FILTER_BW);
	#endif
	#ifdef CENTRE_FREQ
	set_centre_frequency(CENTRE_FREQ);
	#endif
	#ifdef TX_RF_GAIN
	tx_set_rf_gain(TX_RF_GAIN);
	#endif
	#ifdef TX_IF_GAIN
	tx_set_if_gain(TX_IF_GAIN);
	#endif
	#ifdef TX_ANTENNA_ENABLE
	set_antenna_enable(TX_ANTENNA_ENABLE);
	#endif

	// Start transceiver once (puts M0 into TX_START, waiting for SGPIO)
	transceiver_startup(TRANSCEIVER_MODE_TX);

	static uint32_t phase = 0; // Phase tracking for waveform continuity
	uint32_t usb_count = 0;

	baseband_streaming_enable(&sgpio_config);

	while (1) {
		// Top off all available space each pass to maximize producer headroom.
		while ((usb_count - m0_state.m0_count) <=
		       USB_BULK_BUFFER_SIZE - USB_TRANSFER_SIZE) {
			fill_data_buffer(
				&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK],
				USB_TRANSFER_SIZE,
				&phase);

			// M4 publishes produced bytes; M0 advances m0_count as it consumes.
			m0_state.m4_count += USB_TRANSFER_SIZE;
			usb_count += USB_TRANSFER_SIZE;
		}
	}

	transceiver_shutdown();
}
#endif

void off_mode(uint32_t seq)
{
	hackrf_ui()->set_transceiver_mode(TRANSCEIVER_MODE_OFF);

	while (transceiver_request.seq == seq) {}
}
