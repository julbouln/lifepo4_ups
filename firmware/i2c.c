#include "i2c.h"
#include "cmd.h"
#include "config.h"

#ifdef I2C_CMD_ENABLE

void i2c_slave_setup(uint8_t ownaddress)
{
  rcc_periph_clock_enable(RCC_GPIOF);
  rcc_periph_clock_enable(RCC_I2C1);
  rcc_set_i2c_clock_hsi(I2C1);

  // configure i2c pins
  gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO0 | GPIO1);
  gpio_set_output_options(GPIOF, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, GPIO0 | GPIO1);
  gpio_set_af(GPIOF, GPIO_AF1, GPIO0 | GPIO1);

  i2c_reset(I2C1);
  i2c_peripheral_disable(I2C1);

  i2c_enable_analog_filter(I2C1);
  i2c_set_digital_filter(I2C1, 0);

  i2c_set_speed(I2C1, i2c_speed_sm_100k, 8);
  i2c_enable_stretching(I2C1);
  //i2c_disable_stretching(I2C1);
  i2c_set_7bit_addr_mode(I2C1);
  i2c_set_own_7bit_slave_address(I2C1, ownaddress);
  i2c_enable_interrupt(I2C1, I2C_CR1_STOPIE | I2C_CR1_ADDRIE | I2C_CR1_RXIE | I2C_CR1_TXIE | I2C_CR1_NACKIE);

  // enable slave
  I2C1_OAR1 |= I2C_OAR1_OA1EN_ENABLE;

  i2c_peripheral_enable(I2C1);

  nvic_enable_irq(NVIC_I2C1_IRQ);
}

uint8_t slave_recv[16] = {0};
uint8_t slave_recv_len = 0;

uint8_t slave_send[16] = {0};
uint8_t slave_send_len = 0;
uint8_t slave_sent = 0;

//i2c1 ISR
void i2c1_isr(void)
{
  uint32_t isr = I2C_ISR(I2C1);

  // Address matched (Slave)
  if (isr & I2C_ISR_ADDR)
  {

    slave_recv_len = 0;
    slave_sent = 0;

    if (isr & I2C_ISR_DIR_READ)
    {
      I2C1_ICR |= I2C_ISR_TXE; /* flush any data in TXDR */
      I2C1_CR1 |= I2C_CR1_TXIE; /* Set transmit IT */
    }

    // Clear address match flag
    I2C1_ICR |= I2C_ICR_ADDRCF;
  }
  // Receive buffer not empty
  else if (isr & I2C_ISR_RXNE)
  {
    //read bytes from slave
    volatile uint8_t byte = i2c_get_data(I2C1);
    slave_recv[slave_recv_len] = byte;
    slave_recv_len++;
  }
  // Transmit buffer empty & Data byte transfer not finished
  else if ((isr & I2C_ISR_TXIS))
  {
    //send data to master in MSB order
    volatile uint8_t byte = slave_send[slave_sent];
    slave_sent++;
    i2c_send_data(I2C1, byte);

    if (slave_sent >= slave_send_len) {
      I2C1_CR1 &=~ I2C_CR1_TXIE;
    }
  }
  // done by master by sending STOP
  //this event happens when slave is in Recv mode at the end of communication
  else if (isr & I2C_ISR_STOPF)
  {

    //i2c_peripheral_enable(I2C1);
    I2C1_ICR |= I2C_ICR_STOPCF;
   
    I2C1_TXDR = 0; //hack to avoid another txis interrupt at the end
    if (slave_recv_len > 0)
      cmd_parse(slave_recv, slave_recv_len, slave_send, &slave_send_len);

  }
  else if (isr & I2C_ISR_NACKF)
  {
    I2C1_ICR |= I2C_ICR_NACKCF;
  }
}

#endif