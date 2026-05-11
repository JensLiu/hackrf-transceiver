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


// CUSTOM IMPORTS
#include "custom_config.h"
#include "sine_table.h"
#include "gr_lib.h"
#include "mac_protocol.h"

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
#include <stdint.h>
#include <string.h>
#include "uart.h"

#include "usb_endpoint.h"

#include "usb_api_sweep.h"
#include <math.h>
#include "gpio_lpc.h"	
#include "gpio.h"





// Use completely custom names to avoid header conflicts
#define SAFE_DWT_CONTROL  (*((volatile uint32_t*)0xE0001000))
#define SAFE_DWT_CYCCNT   (*((volatile uint32_t*)0xE0001004))
#define SAFE_DEMCR        (*((volatile uint32_t*)0xE000EDFC))

#define SAFE_TRCENA       (1 << 24)
#define SAFE_CYCCNTENA    (1 << 0)

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

volatile bool usb_ready = true;
uint8_t current_tx_buf = 0;
volatile uint32_t usb_overflow_count = 0;

// MAC DATA
static uint8_t* tx_data_ptr = NULL;
static uint32_t tx_data_len = 0;


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
		uart_printf(
			"usb_vendor_request_set_sample_rate_frac DATA freq_hz=%u divider=%u\n",
			set_sample_r_params.freq_hz,
			set_sample_r_params.divider);
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
static uint32_t preamble_bit_index = 0;

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
	static const uint8_t default_pattern[] = CUSTOM_BYTE_PATTERN;
	tx_set_bit_pattern(default_pattern, sizeof(default_pattern));
}

void tx_stream_reset() {
    tx_data_ptr = NULL;
    tx_data_len = 0;
    // also reset byte_index / bit_index / sample_in_bit / table_phase
}


static uint32_t table_phase = 0;
bool fill_data_buffer(uint8_t* buffer, uint32_t len, uint32_t* _unused_phase)
{

    static 	uint32_t sample_in_bit = 0;
    static uint32_t byte_index = 0;
    static uint8_t bit_index = 0;

    const uint32_t BYTES_PER_SAMPLE = 2;
    const uint8_t MID_SCALE = 0;
    const uint32_t SINE_TABLE_MASK = SINE_TABLE_SIZE - 1;
    const uint32_t phase_increment =
        (SINE_FREQ * SINE_TABLE_SIZE) / TX_SAMPLE_RATE;

    uint32_t total_samples = len / BYTES_PER_SAMPLE;
    
	for (uint32_t s = 0; s < total_samples; s++) {

        uint32_t i = s * BYTES_PER_SAMPLE;

        // ===== STREAM BIT FROM BYTE BUFFER =====
        uint8_t current_byte = tx_data_ptr[byte_index];
        bool bit = (current_byte >> (7 - bit_index)) & 0x01;
		
        // ===== MODULATION =====
        if (bit) {
            buffer[i] = sine_table[2 * table_phase];
            buffer[i + 1] = sine_table[2 * table_phase + 1];
        } else {
            buffer[i] = MID_SCALE;
            buffer[i + 1] = MID_SCALE;
        }

        // ===== PHASE UPDATE =====
        table_phase = (table_phase + phase_increment) & SINE_TABLE_MASK;

        // ===== BIT TIMING =====
        sample_in_bit++;

        if (sample_in_bit >= TX_BIT_SAMPLES) {
            sample_in_bit = 0;

            bit_index++;

            if (bit_index >= 8) {
                bit_index = 0;
                byte_index++;

                // STOP CONDITION (end of TX buffer)
                if (byte_index >= tx_data_len) {
					byte_index = 0;
					bit_index = 0;
					table_phase = 0;
					sample_in_bit = 0;
                    return true;
                }
            }
        }
    }
	return false;
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
		//led_off(LED3);
		//led_on(LED2);
		m0_set_mode(M0_MODE_RX);
		m0_state.shortfall_limit = _rx_overrun_limit;
		break;
	case TRANSCEIVER_MODE_TX:
		//led_off(LED2);
		//led_on(LED3);
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
		// uart_printf(
		// 	"usb_vendor_request_set_transceiver_mode SETUP value=%u\n",
		// 	endpoint->setup.value);
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
	usb_ready = true;
}

float _rx_lut[0xff];

void init_rx_lookup_table(void)
{
	for (unsigned int i = 0; i <= 0xff; i++) {
		_rx_lut[i] = ((float) ((int8_t) i)) * (1.0f / 128.0f);
	}
	uart_printf("RX Lookup Table initialised\n");
}

void off_mode(uint32_t seq)
{
	hackrf_ui()->set_transceiver_mode(TRANSCEIVER_MODE_OFF);

	while (transceiver_request.seq == seq) {}
}

void phy_tx_step(packet_t* pkt_out, uint8_t dst)
{

    init_sine_table();
    // === Build MAC frame (BYTE STREAM, not bit stream) ===
    if (tx_data_ptr == NULL){

        mac_build_frame(
            dst,
            (mac_device_id + 1) % N,
            pkt_out->len,
            pkt_out->data
        );

        // IMPORTANT:
        // Instead of expanding to bits, we only store byte stream info
		
        tx_data_ptr = mac_get_tx_data(&tx_data_len); 
  
		// tx_data_ptr should point to raw bytes (NOT bits)
    }

    // === Radio config ===
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

    transceiver_startup(TRANSCEIVER_MODE_TX);



    static uint32_t phase = 0;
    uint32_t usb_count = 0;


    baseband_streaming_enable(&sgpio_config);
	led_off(LED2);
	led_on(LED3);

	//delay(20000000); // add a delay to see the transition between tx and rx on the LED of the hackrf one

    // === STREAMING MODE (NO PRECOMPUTED BITS) ===
    const uint32_t total_samples = tx_data_len * 8 * TX_BIT_SAMPLES;

    uint32_t samples_sent = 0;
	bool done = false;



		done = false;
		samples_sent=0;
		
		while (!done && samples_sent < total_samples) {
			if ((usb_count - m0_state.m0_count) <=
				USB_BULK_BUFFER_SIZE - USB_TRANSFER_SIZE) {

				done = fill_data_buffer(
					&usb_bulk_buffer[usb_count & USB_BULK_BUFFER_MASK],
					USB_TRANSFER_SIZE,
					&phase
				);

				m0_state.m4_count += USB_TRANSFER_SIZE;
				usb_count += USB_TRANSFER_SIZE;

				samples_sent += USB_TRANSFER_SIZE / 2; // 2 bytes per sample
			}
		}

		while (m0_state.m0_count < usb_count) {
			// spin or optionally sleep
		}


	tx_stream_reset();
	//while(1);
    transceiver_shutdown();

}



bool phy_rx_step(packet_t* pkt_in, mac_frame_t* frame)
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
	#ifdef RX_BB_GAIN
	rx_set_bb_gain(RX_BB_GAIN);
	#endif
	#ifdef RX_ANTENNA_ENABLE
	set_antenna_enable(RX_ANTENNA_ENABLE);
	#endif

	uint32_t consume_count = 0;
	transceiver_startup(TRANSCEIVER_MODE_RX);
	baseband_streaming_enable(&sgpio_config);

	// Constants matching the transmitter
	uint32_t sample_count = 0;
	uint64_t mag2_sum = 0; // Sum of magnitude squared
	static uint8_t bit_buffer[256];

	uint32_t bit_buffer_index = 0;
	uint8_t tx_buffer[RX_BIT_PACKET_SIZE_DEBUG]; // Separate buffer for USB transfers
	uint32_t noise_floor = 0;
	uint8_t preamble_window_detector = 0;

	uint32_t synchronization_counter = 0;

	// State Machine 
	typedef enum {
		STATE_IDLE,        // Looking for 10101010 pattern
		STATE_REFINE_SYNC, // Pattern found, now finding the exact peak
		STATE_LOCKED       // Synchronized, reading data bits
	} rx_state_t;

	rx_state_t state = STATE_IDLE;

	// Integration variables
	uint32_t sliding_sum = 0;
	uint32_t max_energy_found = 0;
	uint32_t samples_since_max = 0;


	uint16_t first_word = 0;
	bool synchronized = false;
	bool data_received = true;
	uint8_t bit_count = 0;
	uint8_t byte_acc = 0;

	static bool samples_dropped = false;
	static uint32_t max_backlog = 0;
	static uint32_t total_iq_samples = 0;
	led_on(LED2);
	led_off(LED3);
	uint32_t waiting_for_preamble_since = 0;
	while (data_received) {


		uint32_t backlog = m0_state.m0_count - consume_count;

		if (backlog > max_backlog) {
			max_backlog = backlog;
		}

		if (backlog > USB_BULK_BUFFER_SIZE) {
			samples_dropped = true;
		}

		if(bit_buffer_index == 16){
			samples_dropped = true;
		}

		if ((m0_state.m0_count - consume_count) >= BATCH_SAMPLE_SIZE) {
			uint8_t* buffer = &usb_bulk_buffer[consume_count & USB_BULK_BUFFER_MASK];

			for (uint32_t i = 0; i < BATCH_SAMPLE_SIZE; i += 2) {
				int32_t I = (int8_t) buffer[i];
				int32_t Q = (int8_t) buffer[i + 1];
				uint32_t mag2 = I * I + Q * Q;
			
				switch (state) {
					case STATE_IDLE:
					mag2_sum += mag2;
					sample_count++;

					// 1. Track edges at the bit level with no synchronization
					if (sample_count >= RX_BIT_SAMPLES) {
						const uint32_t mag2_avg = mag2_sum >> RX_BIT_SHIFT;
						noise_floor = (noise_floor * 15 + mag2_avg) >> 4;
						const uint32_t adaptive_threshold = noise_floor + RX_THRESHOLD_MARGIN;
						bool current_bit = (mag2_avg > adaptive_threshold);
						//bit_buffer[bit_buffer_index++] = current_bit;
						
						preamble_window_detector = (preamble_window_detector << 1) | current_bit;

						//bit_buffer[bit_buffer_index++] = preamble_window_detector_A & 1;
						sample_count = 0;
						mag2_sum = 0;


						// Once we've seen enough transitions (8 bits of 10101010)
						if (preamble_window_detector == 0xAA) {
						state = STATE_REFINE_SYNC;
						max_energy_found = 0;
						samples_since_max = 0;

						synchronization_counter = 0;
						sample_count = 0;
						mag2_sum = 0;
						sliding_sum=0;
						synchronized = false;
						break;
						} 

						waiting_for_preamble_since++;
						if(5000 < waiting_for_preamble_since)
						{
							return false;   // no preamble arrived in time
						}

					}
				break;

				case STATE_REFINE_SYNC:
					
					mag2_sum += mag2;
					sample_count++;
					synchronization_counter += 1; // make sure that 8 bits are passed

					// 2. Find the EXACT peak energy center of the NEXT pulse
					if (sliding_sum > max_energy_found) {
						max_energy_found = sliding_sum;
						samples_since_max = 0;
					} else {
						samples_since_max+=1;
					}
					
					//bit_buffer[bit_buffer_index++] = current_bit;
					//bit_buffer[bit_buffer_index++] = 1; //Debug

					if (sample_count >= RX_BIT_SAMPLES) {


						const uint32_t mag2_avg = mag2_sum >> RX_BIT_SHIFT;

						
						noise_floor = (noise_floor * 15 + mag2_avg) >> 4;
						const uint32_t adaptive_threshold = noise_floor + RX_THRESHOLD_MARGIN;
						bool current_bit;
						current_bit = (mag2_avg > adaptive_threshold);



						//bit_buffer[bit_buffer_index++] = current_bit;
						sample_count = 0;
						mag2_sum = 0;
						first_word = (first_word << 1) | current_bit;
						if((first_word == 0xB5D2) && (synchronized == true)){
							state = STATE_LOCKED;
							bit_count = 0;
							byte_acc = 0;
						}


					}

					// After we pass the peak and it drops slightly (50 sample confirm)
					
					if ((samples_since_max > 250) && (synchronization_counter > 16 * RX_BIT_SAMPLES) && (synchronized == false)) { // 
						// SYNC OFFSET: We are 'samples_since_max' past the peak.
						// We want our next bit window to start exactly half a bit after the peak.
						// Or simply reset sample_count to align to this peak center.
						sample_count = samples_since_max % RX_BIT_SAMPLES; 
						mag2_sum = 0;
						synchronized = true;
						noise_floor = max_energy_found >> (RX_BIT_SHIFT+1);
					}

				break;

				case STATE_LOCKED:
					mag2_sum += mag2;
					sample_count++;
					if (sample_count >= RX_BIT_SAMPLES) {
						const uint32_t mag2_avg = mag2_sum >> RX_BIT_SHIFT;
						//noise_floor = (noise_floor * 15 + mag2_avg) >> 4;
						bool current_bit = (mag2_avg > noise_floor);
						//bit_buffer[bit_buffer_index++] = current_bit;
						
						byte_acc = (byte_acc << 1) | current_bit;
						bit_count++;
 						if (bit_count == 8) {

							// High-precision bit decision
							bit_buffer[bit_buffer_index++] = byte_acc;

 							if (mac_process_byte(byte_acc, frame)) {
								data_received = false;
								//return true; // frame complete
								//bit_buffer[bit_buffer_index] = (uint8_t) frame->payload_size;
								//bit_buffer_index++;
							}
/* 							if(bit_buffer_index == 16){
								data_received = false;
							} */

							bit_count = 0;
							byte_acc = 0;
						}
						sample_count = 0;
						mag2_sum = 0;
					}
				break;
			}
			}
		consume_count += BATCH_SAMPLE_SIZE;
		m0_state.m4_count += BATCH_SAMPLE_SIZE;
		}

		

	}

/* 	for(uint32_t i =0 ; i < frame->payload_size ; i){
		memcpy(tx_buffer, &frame->payload[i], RX_BIT_PACKET_SIZE_DEBUG);
				usb_transfer_schedule_block(
					&usb_endpoint_bulk_in,
					tx_buffer,
					RX_BIT_PACKET_SIZE_DEBUG,
					transceiver_bulk_transfer_complete,
					NULL);
				while (!usb_endpoint_bulk_in.transfer_complete) {
					__asm__("nop");
					}
				i+=RX_BIT_PACKET_SIZE_DEBUG;
	} */



	if(frame->next_tx == mac_device_id){
		return true;

	}
	else{
		return false;
	}

	transceiver_shutdown();
}

