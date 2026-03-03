#include "uart.h"

char uart_read_char(void) { return (char)uart_read(UART0); }

void uart_send_str(const char *a) { // send string *a to uart - BLOCKING + ATOMIC
  // Disable interrupts for atomic output
  uint32_t primask;
  __asm__ volatile("mrs %0, primask" : "=r"(primask));
  __asm__ volatile("cpsid i" ::: "memory");
  
  int i = 0;
  while (a[i] != 0) {
    // Wait for TX FIFO to have space (THRE = Transmitter Holding Register Empty)
    while (!(UART_LSR(UART0) & UART_LSR_THRE)) {}
    uart_write(UART0, a[i]);
    i++;
  }
  
  // Wait for transmission to complete before re-enabling interrupts
  while (!(UART_LSR(UART0) & UART_LSR_TEMT)) {}
  
  // Restore interrupt state
  __asm__ volatile("msr primask, %0" :: "r"(primask) : "memory");
}

bool rf_uart_receive(char *data, const int max_len) {
  for (int i = 0; i < max_len; i++) {
    data[i] = uart_read_char();
  }
  return true;
}

void rf_uart_send(const char *data, const int len) {
  for (int i = 0; i < len; i++) {
    // // Optional: wait for TX FIFO to have space
    // while (!(UART0_LSR & UART_LSR_THRE)) {
    // }
    uart_write(UART0, data[i]);
  }
}

bool get_uart_rate_config(int uart_rate, uint16_t *uart_divisor,
                          uint8_t *uart_divaddval, uint8_t *uart_mulval) {
  if (115200 == uart_rate) {
    *uart_divisor = 83;
    *uart_divaddval = 1;
    *uart_mulval = 3;
    return (true);
  }

  if (921600 == uart_rate) {
    *uart_divisor = 9;
    *uart_divaddval = 7;
    *uart_mulval = 13;
    return (true);
  }

  if (9600 == uart_rate) {
    *uart_divisor = 1033;
    *uart_divaddval = 2;
    *uart_mulval = 7;
    return (true);
  }

  return (false);
}

void delay_1us(const uint32_t num_1us) {
  uint32_t i;
  const uint32_t duration = num_1us * WAIT_CPU_CLOCK_INIT_DELAY;

  for (i = 0; i < duration; i++)
    __asm__("nop");
}

void uart_pin_setup() {
  /* Configure UART0 PIN*/
  /* TX: Function 1, No special pull-up/down (default to drive), High speed slew
   * rate if possible */
  /* SCU_PINMUX_U0_TXD (P2_0) -> UART0_TXD (Func 1) */
  scu_pinmux(SCU_PINMUX_U0_TXD, SCU_CONF_EPUN_DIS_PULLUP | SCU_CONF_FUNCTION1);

  /* RX: Function 1, Pull-up enabled for idle high stability */
  /* SCU_PINMUX_U0_RXD (P2_1) -> UART0_RXD (Func 1) */
  scu_pinmux(SCU_PINMUX_U0_RXD, SCU_CONF_EZI_EN_IN_BUFFER | SCU_CONF_FUNCTION1);
}

void uart_setup() {
  /* Re-enable UART0 clock */
  CGU_BASE_UART0_CLK = CGU_BASE_UART0_CLK_AUTOBLOCK(1) |
                       CGU_BASE_UART0_CLK_CLK_SEL(CGU_SRC_PLL1);
  CCU1_CLK_M4_USART0_CFG = 1;

  uint16_t uart_divisor;
  uint8_t uart_divaddval, uart_mulval;
  get_uart_rate_config(921600, &uart_divisor, &uart_divaddval, &uart_mulval);
  uart_init(UART0, UART_DATABIT_8, UART_STOPBIT_1, UART_PARITY_NONE,
            uart_divisor, uart_divaddval, uart_mulval);

  // Explicitly Enable FIFO
  UART_FCR(UART0) =
      UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV0;
}

char DISPLAY_BUFFER[512];

int uart_printf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  int n = vsnprintf(DISPLAY_BUFFER, sizeof(DISPLAY_BUFFER), format, args);
  va_end(args);
  if (n > 0)
    uart_send_str(DISPLAY_BUFFER);
  return n;
}