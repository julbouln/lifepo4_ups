#include "adc.h"
#include "config.h"

/* BAT MONITOR */

static inline __attribute__((always_inline)) void __NOP(void)
{
	__asm volatile("nop");
}

#define FREQUENCY 48000000

static void wait(int ms)
{
	int w = FREQUENCY / 10000 * ms;
	int l;
	for (l = 0; l < w; l++)
	{
		__NOP();
	}
}

void adc_setup(void)
{
	adc_power_off(ADC1);
	adc_set_clk_source(ADC1, ADC_CLKSOURCE_ADC);
	adc_calibrate(ADC1);
	adc_set_operation_mode(ADC1, ADC_MODE_SCAN);
	adc_disable_external_trigger_regular(ADC1);
	adc_set_right_aligned(ADC1);
	adc_disable_vrefint();
	adc_disable_temperature_sensor();
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_071DOT5);
	adc_set_resolution(ADC1, ADC_RESOLUTION_12BIT);
	adc_disable_analog_watchdog(ADC1);
	adc_power_on(ADC1);

	/* Wait for ADC starting up. */
    // FIXME: is this really needed ?
	wait(200);
}

uint32_t adc_read(uint8_t chan)
{
	uint32_t v;
	uint8_t channels[1];

	channels[0] = chan;

	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)))
		;

	v = adc_read_regular(ADC1);
	return v;
}

static float sense(uint8_t chan)
{
	int i;
	float s = 0.0;
	for (i = 0; i < ADC_SAMPLES; i++)
	{
		s += (float)adc_read(chan);
	}
	return (s / ADC_SAMPLES) / 4095.0;
};

float bat_sense_a()
{
	return sense(1);
}

float bat_sense_v()
{
	return sense(2);
}

float inp_sense_a()
{
	return sense(6);
}
