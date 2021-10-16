#ifndef ADC_H
#define ADC_H

#include <libopencm3/stm32/adc.h>

void adc_setup(void);
float bat_sense_v(void);
float bat_sense_a(void);
float inp_sense_a(void);

#endif