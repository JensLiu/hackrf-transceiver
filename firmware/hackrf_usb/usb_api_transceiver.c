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

#define USB_TRANSFER_SIZE 0x4000

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

void off_mode(uint32_t seq)
{
	hackrf_ui()->set_transceiver_mode(TRANSCEIVER_MODE_OFF);

	while (transceiver_request.seq == seq) {}
}
