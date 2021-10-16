#ifndef POWER_H
#define POWER_H

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/cm3/scb.h>

void scb_enable_deep_sleep_mode(void);
void power_down_delay(int min, int sec);
void power_down_postpone(void);
void power_down_check_postpone(void);

void charge_enable();
void charge_disable();
void boost_enable();
void boost_disable();
void power_up(void);
void power_down(void);
void standby(void);

#endif