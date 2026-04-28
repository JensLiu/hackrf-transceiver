#include "custom_transceiver.h"
#include "custom_config.h"
#include "gr_lib.h"
#include "uart.h"

#include "hackrf_core.h"
#include "hackrf_ui.h"
#include "m0_state.h"
#include "operacake_sctimer.h"
#include "radio.h"
#include "streaming.h"
#include <libopencm3/lpc43xx/m4/nvic.h>

#include <math.h>

// receiver state
uint32_t rx_consume_count = 0;
uint32_t rx_sample_count = 0;
uint64_t rx_mag2_sum = 0;
bool rx_current_bit = false;
uint8_t rx_bit_buffer[RX_BIT_PACKET_SIZE / 8]; // Buffer to store detected bits
uint32_t rx_bit_index = 0;
uint32_t rx_bit_buffer_index = 0;
uint32_t rx_noise_floor = 0;

void custom_transceiver_receive_begin() {
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
  transceiver_startup(TRANSCEIVER_MODE_RX);
  baseband_streaming_enable(&sgpio_config);
  rx_consume_count = 0;
  rx_sample_count = 0;
  rx_mag2_sum = 0;
  rx_current_bit = false;
  rx_bit_index = 0;
  rx_bit_buffer_index = 0;
  rx_noise_floor = 0;
  tusb_uart_printf("Custom transceiver receive started\r\n");
}

void custom_transceiver_receive_end() {
  transceiver_shutdown();
  rx_consume_count = 0;
  rx_sample_count = 0;
  rx_mag2_sum = 0;
  rx_current_bit = false;
  rx_bit_index = 0;
  rx_bit_buffer_index = 0;
  rx_noise_floor = 0;
  tusb_uart_printf("Custom transceiver receive stopped\r\n");
}

void custom_transceiver_receive() {

  if ((m0_state.m0_count - rx_consume_count) >= BATCH_SAMPLE_SIZE) {
    uint8_t *buffer = &usb_bulk_buffer[rx_consume_count & USB_BULK_BUFFER_MASK];

#if BATCH_SAMPLE_SIZE > 2 && BATCH_SAMPLE_SIZE % 2 == 0
#define BATCH_SAMPLE_LOOP
#elif BATCH_SAMPLE_SIZE != 2
#error "BATCH_SAMPLE_SIZE must be at least 2"
#endif

#ifdef BATCH_SAMPLE_LOOP
    for (uint32_t i = 0; i < BATCH_SAMPLE_SIZE; i += 2) {
      int32_t I = (int8_t)buffer[i];
      int32_t Q = (int8_t)buffer[i + 1];
#else
    int32_t I = (int8_t)buffer[0];
    int32_t Q = (int8_t)buffer[1];
#endif
      const uint32_t mag2 = I * I + Q * Q;
      //   uart_printf("%d\t", mag2);
      rx_mag2_sum += mag2;
      rx_sample_count++;

      if (rx_sample_count >= RX_BIT_SAMPLES) {
        const uint32_t mag2_avg = rx_mag2_sum / RX_BIT_SAMPLES;
        rx_noise_floor = (rx_noise_floor * 15 + mag2_avg) / 16;
        const uint32_t adaptive_threshold =
            rx_noise_floor + RX_THRESHOLD_MARGIN;
        rx_current_bit = (mag2_avg > adaptive_threshold);
#ifdef DEBUG_RX_PRINT_DECODED_BITS
        uart_printf("%d", rx_current_bit);
#endif
#ifdef RX_FTDI_SEND_IN_BATCH
        {
          // Start a fresh byte every 8 bits.
          if ((rx_bit_index & 0x07u) == 0u) {
            rx_bit_buffer_index = (rx_bit_index >> 3);
            rx_bit_buffer[rx_bit_buffer_index] = 0u;
          }

          // Shift left and append the new bit into LSB.
          rx_bit_buffer[rx_bit_buffer_index] =
              (uint8_t)((rx_bit_buffer[rx_bit_buffer_index] << 1) |
                        (rx_current_bit ? 1u : 0u));

          rx_bit_index++;

          // When RX_BIT_PACKET_SIZE bits are packed, send them
          if (rx_bit_index >= RX_BIT_PACKET_SIZE) {
#ifdef DEBUG_RX_FTDI_PRINT_SEND_BITS
            uart_printf("\r\n");
            uart_printf("Sending...\r\n");
            for (uint32_t i = 0; i < RX_BIT_PACKET_SIZE / 8; i++) {
              for (int bit = 7; bit >= 0; bit--) {
                uart_printf("%d", (rx_bit_buffer[i] >> bit) & 0x01u);
              }
            }
            uart_printf("\r\n");
#endif
#ifdef RX_FTDI_BLOCKING_IO
            const uint32_t write_count = ftdi_host_write_blocking(
                rx_bit_buffer, RX_BIT_PACKET_SIZE / 8, FTDI_IO_TIMEOUT_MS);
            if (write_count == RX_BIT_PACKET_SIZE / 8) {
              uart_printf(".");
            } else {
              uart_printf("Failed to send batch to FTDI\r\n");
            }
#endif
            rx_bit_buffer_index = 0;
            rx_bit_index = 0;
          }
        }
#endif
        rx_sample_count = 0;
        rx_mag2_sum = 0;
      }

#ifdef BATCH_SAMPLE_LOOP
    }
#endif
    rx_consume_count += BATCH_SAMPLE_SIZE;
    m0_state.m4_count += BATCH_SAMPLE_SIZE;
  }
}