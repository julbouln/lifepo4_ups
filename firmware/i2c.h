#ifndef I2C_H
#define I2C_H
#include <stdlib.h>

#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/cm3/nvic.h>

void i2c_slave_setup(uint8_t ownaddress);

#endif