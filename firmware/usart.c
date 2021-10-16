#include "config.h"
#include "usart.h"
#include "cmd.h"

#ifndef USB_ENABLE
void usart_setup(void)
{
	/* Enable clocks for USART1. */
	rcc_periph_clock_enable(RCC_USART1);

	/* Setup GPIO pins for USART1 transmit. */
	gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX_PIN | USART_RX_PIN);

	/* Setup USART1 TX pin as alternate function. */
	gpio_set_af(USART_PORT, GPIO_AF1, USART_TX_PIN | USART_RX_PIN);

	/* Setup USART1 parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_stopbits(USART1, USART_CR2_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART1);

	// Enable interrupts for tx/rx
	nvic_enable_irq(NVIC_USART1_IRQ);
  	usart_enable_rx_interrupt(USART1);
	// usart_enable_tx_interrupt(USART1);
}

uint8_t rx_len = 0;
uint8_t rx_buffer[16] = {0};

uint8_t tx_len = 0;
uint8_t tx_buffer[16] = {0};

uint8_t tx_sent = 0;

void usart1_isr(void)
{
// Check if interrupt is signalling TX ready
  if (usart_get_flag(USART1, USART_FLAG_TXE)) {
  	if(tx_len > 0) {
  	  uint8_t c = tx_buffer[tx_sent];
      usart_send(USART1, c);
      tx_sent++;
      tx_len--;
  	} else {
  	  tx_sent = 0; // reset tx_buffer
      // If there's no more data to be sent, disable this interrupt
      usart_disable_tx_interrupt(USART1);  		
  	}
  }

  // Check if data has been received (RX Not Empty)
  if (usart_get_flag(USART1, USART_FLAG_RXNE)) {
    const uint8_t c = usart_recv(USART1);
    rx_buffer[rx_len] = c;
  	rx_len++;
  }

  if(rx_len > 0) {
	cmd_parse(rx_buffer, rx_len, tx_buffer, &tx_len);
	if(tx_len > 0) { // command was parsed
		rx_len = 0; // reset rx buffer
		usart_enable_tx_interrupt(USART1);
	}
  }
}
#endif