#include "hackrf_core.h"
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/lpc43xx/scu.h>
#include <libopencm3/lpc43xx/uart.h>
#include <stdio.h>
#include <stdarg.h> // < va_list, va_start, va_end

#define WAIT_CPU_CLOCK_INIT_DELAY (34) // 1us

extern char DISPLAY_BUFFER[512];

void uart_pin_setup();
void uart_setup();
char uart_read_char(void);
void uart_send_str(const char *a);
void delay_1us(const uint32_t num_1us);
void rf_uart_send(const char *data, const int len);
bool rf_uart_receive(char *data, const int max_len);
int uart_printf(const char *format, ...);