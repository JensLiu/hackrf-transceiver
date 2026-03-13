
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

// CUSTOM IMPORTS
#include "custom_config.h"
#include "uart.h"
#include "gr_lib.h"

extern uint32_t __m0_start__;
extern uint32_t __m0_end__;
extern uint32_t __ram_m0_start__;
extern uint32_t _etext_ram, _text_ram, _etext_rom;
extern void tx_autostart_init(void);

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
		if (baseband_filter_bandwidth_set(bandwidth)) {
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
		if (set_freq(freq)) {
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
		if (set_freq_explicit(
			    explicit_params.if_freq_hz,
			    explicit_params.lo_freq_hz,
			    explicit_params.path)) {
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
		if (sample_rate_frac_set(
			    set_sample_r_params.freq_hz * 2,
			    set_sample_r_params.divider)) {
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
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uart_printf(
			"usb_vendor_request_set_amp_enable SETUP value=%u\n",
			endpoint->setup.value);
		switch (endpoint->setup.value) {
		case 0:
			rf_path_set_lna(&rf_path, 0);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			rf_path_set_lna(&rf_path, 1);
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
		uint8_t value;
		value = max283x_set_lna_gain(&max283x, endpoint->setup.index);
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
		uint8_t value;
		value = max283x_set_vga_gain(&max283x, endpoint->setup.index);
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
		uint8_t value;
		value = max283x_set_txvga_gain(&max283x, endpoint->setup.index);
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
	if (stage == USB_TRANSFER_STAGE_SETUP) {
		uart_printf(
			"usb_vendor_request_set_antenna_enable SETUP value=%u\n",
			endpoint->setup.value);
		switch (endpoint->setup.value) {
		case 0:
			rf_path_set_antenna(&rf_path, 0);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		case 1:
			rf_path_set_antenna(&rf_path, 1);
			usb_transfer_schedule_ack(endpoint->in);
			return USB_REQUEST_STATUS_OK;
		default:
			return USB_REQUEST_STATUS_STALL;
		}
	} else {
		return USB_REQUEST_STATUS_OK;
	}
}

static volatile hw_sync_mode_t _hw_sync_mode = HW_SYNC_MODE_OFF;
static volatile uint32_t _tx_underrun_limit;
static volatile uint32_t _rx_overrun_limit;

void set_hw_sync_mode(const hw_sync_mode_t new_hw_sync_mode)
{
	_hw_sync_mode = new_hw_sync_mode;
}

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
	rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_OFF);
	m0_set_mode(M0_MODE_IDLE);
}

void transceiver_startup(const transceiver_mode_t mode)
{
	hackrf_ui()->set_transceiver_mode(mode);

	switch (mode) {
	case TRANSCEIVER_MODE_RX_SWEEP:
	case TRANSCEIVER_MODE_RX:
		led_off(LED3);
		led_on(LED2);
		rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_RX);
		m0_set_mode(M0_MODE_RX);
		m0_state.shortfall_limit = _rx_overrun_limit;
		break;
	case TRANSCEIVER_MODE_TX:
		led_off(LED2);
		led_on(LED3);
		rf_path_set_direction(&rf_path, RF_PATH_DIRECTION_TX);
		m0_set_mode(M0_MODE_TX_START);
		m0_state.shortfall_limit = _tx_underrun_limit;
		break;
	default:
		break;
	}

	activate_best_clock_source();
	hw_sync_enable(_hw_sync_mode);
}

usb_request_status_t usb_vendor_request_set_transceiver_mode(
	usb_endpoint_t* const endpoint,
	const usb_transfer_stage_t stage)
{
	if (stage == USB_TRANSFER_STAGE_SETUP) {
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
		set_hw_sync_mode(endpoint->setup.value);
		usb_transfer_schedule_ack(endpoint->in);
		return USB_REQUEST_STATUS_OK;
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
void rx_mode(uint32_t seq)
{
	// sample_rate_frac_set(TX_SAMPLE_RATE, 1);
	// // baseband_filter_bandwidth_set(10000);
	// set_freq(CENTRE_FREQ);
	// max283x_set_lna_gain(&max283x, 40);
	// rf_path_set_lna(&rf_path, 1);
	// rf_path_set_antenna(&rf_path, 1);
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
	// sample_rate_frac_set(SAMPLE_RATE, 1); // 20 MS/s
	// // baseband_filter_bandwidth_set(15000000);   // 15 MHz bandwidth
	// set_freq(FREQ);                       // Frequency 915 MHz
	// max283x_set_txvga_gain(&max283x, 47); // Maximum TX gain
	// rf_path_set_lna(&rf_path, 1);         // Enable LNA
	// rf_path_set_antenna(&rf_path, 1);     // Select antenna path
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
