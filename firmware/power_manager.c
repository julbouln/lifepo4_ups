/*
 * Based on stm32l-discovery button-irq-printf-lowpower from libopencm3-example
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>

#include <libopencm3/stm32/timer.h>

#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/syscfg.h>

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>

#include <libopencm3/stm32/adc.h>

#include "config.h"
#include "usb.h"
#include "usart.h"
#include "i2c.h"
#include "rtc.h"
#include "pwm.h"
#include "cmd.h"

void scb_enable_deep_sleep_mode(void);
void gpio_setup(void);
void gpio_irq_setup(void);
void wait(int ms);
void power_down_delay(int min, int sec);
void power_down_postpone(void);
void rtc_setup(void);
void rcc_clock_setup_in_hsi_out_8mhz(void);
void clock_setup(void);
void standby(void);
void adc_setup(void);
float bat_sense(void);
void led_set_mode(uint8_t mode);
void led_set_luminance(float l);
void power_up(void);
void power_down(void);

static inline __attribute__((always_inline)) void __WFI(void)
{
	__asm volatile ("wfi");
}

static inline __attribute__((always_inline)) void __NOP(void)
{
	__asm volatile ("nop");
}

void scb_enable_deep_sleep_mode(void)
{
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
}

void gpio_setup(void)
{
	/* WAKE UP/EN pin */
	gpio_mode_setup(EN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, EN_PIN);

	/* SHUTDOWN pin */
	gpio_mode_setup(SHUTDOWN_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SHUTDOWN_PIN);

	/* POWER pin */
	gpio_mode_setup(BOOST_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BOOST_EN_PIN);

	/* UPS POWER pin */
	gpio_mode_setup(CHARGE_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CHARGE_EN_PIN);

	/* BATTERY SENSE V pin */
	gpio_mode_setup(BAT_SENSE_V_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BAT_SENSE_V_PIN);

	/* BATTERY SENSE A pin */
	gpio_mode_setup(BAT_SENSE_A_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BAT_SENSE_A_PIN);

	/* INPUT SENSE A pin */
	gpio_mode_setup(INP_SENSE_A_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, INP_SENSE_A_PIN);

	/* PFO pin */
	/* LOW = on battery, HIGH = on charger */
	gpio_mode_setup(BAT_PFO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BAT_PFO_PIN);

	/* FAULT pin */
	/* LOW = fault, HIGH = OK */
	gpio_mode_setup(BAT_FAULT_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BAT_FAULT_PIN);

	/* CHRG pin */
	/* LOW = charging, HIGH = not charging */
	gpio_mode_setup(BAT_CHRG_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BAT_CHRG_PIN);

}

// WAKE UP pin and ALIVE pin should postpone shut down via IRQ
void gpio_irq_setup() {
	// EXTI0_1 for WAKE UP interrupt
	exti_select_source(EXTI0, GPIOA);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI0);
	nvic_enable_irq(NVIC_EXTI0_1_IRQ);

	// EXTI4_15 for ALIVE interrupt
	exti_select_source(EXTI5, GPIOA);
	exti_set_trigger(EXTI5, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI5);
	nvic_enable_irq(NVIC_EXTI4_15_IRQ);
}

#define FREQUENCY 48000000

void wait(int ms) {
	int w = FREQUENCY / 10000 * ms;
	int l;
	for (l = 0; l < w; l++) {
		__NOP();
	}
}

/* set next alarm: current time + min:sec */
void power_down_delay(int min, int sec)
{
	struct rtc_alarm alarm = {0};
	struct rtc_time time = {0};
	int val;

	/* read current min/sec values */
	rtc_read_calendar(&time, 0);

	/* BCD min/sec math: calculate next alarm */
	val = time.su + sec;
	alarm.su = val % 10;

	val = time.st + val / 10;
	alarm.st = val % 6;

	val = time.mnu + min + val / 6;
	alarm.mnu = val % 10;

	val = time.mnt + val / 10;
	alarm.mnt = val % 6;

	/* this app cares about min/sec only */
	alarm.msk1 = 0;
	alarm.msk2 = 0;
	alarm.msk3 = 1;
	alarm.msk4 = 1;

	/* set new alarm */
	rtc_set_alarm(&alarm);
}

void rtc_setup(void)
{
	/* reset RTC */
	rcc_periph_reset_pulse(RST_BACKUPDOMAIN);

	/* enable LSI clock */
	rcc_osc_on(RCC_LSI);
	rcc_wait_for_osc_ready(RCC_LSI);

	/* select LSI clock for RTC */
	rtc_unlock();
	rcc_set_rtc_clock_source(RCC_LSI);
	rcc_enable_rtc_clock();
	rtc_lock();
}

void flash_prefetch_buffer_enable(void)
{
	FLASH_ACR |= FLASH_ACR_PRFTBE;
}

void rcc_clock_setup_in_hsi_out_8mhz(void)
{
	rcc_osc_on(RCC_HSI);
	rcc_wait_for_osc_ready(RCC_HSI);
	rcc_set_sysclk_source(RCC_HSI);

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
	rcc_set_ppre(RCC_CFGR_PPRE_NODIV);

	flash_prefetch_buffer_enable();
	flash_set_ws(FLASH_ACR_LATENCY_024_048MHZ);

	/* 8MHz * 2 / 2 = 8MHz */
	rcc_set_pll_multiplication_factor(RCC_CFGR_PLLMUL_MUL2);
	rcc_set_pll_source(RCC_CFGR_PLLSRC_HSI_CLK_DIV2);

	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	rcc_set_sysclk_source(RCC_PLL);

	rcc_apb1_frequency = 8000000;
	rcc_ahb_frequency = 8000000;
}

void clock_setup(void)
{

#ifdef USB_ENABLE
	rcc_clock_setup_in_hsi48_out_48mhz();
	rcc_periph_clock_enable(RCC_SYSCFG_COMP);
	SYSCFG_CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
	crs_autotrim_usb_enable();
	rcc_set_usbclk_source(RCC_HSI48);
#else
//	rcc_clock_setup_in_hsi_out_48mhz();
	rcc_clock_setup_in_hsi_out_8mhz();
#endif


	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_RTC);
	rcc_periph_clock_enable(RCC_PWR);
	rcc_periph_clock_enable(RCC_ADC);

	/* enable rtc unlocking */
	pwr_disable_backup_domain_write_protect();
}

void standby() {
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  	gpio_mode_setup(GPIOB, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);
  	gpio_mode_setup(GPIOF, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_ALL);

  	scb_enable_deep_sleep_mode();
	pwr_clear_wakeup_flag();
	pwr_clear_standby_flag();
	pwr_set_standby_mode();
	pwr_disable_power_voltage_detect();
	pwr_enable_wakeup_pin();
	__WFI();
}

#if 1

/* BAT MONITOR */

//uint8_t channel_array[] = { 1, 2, ADC_CHANNEL_TEMP};

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
	wait(200);
}

uint32_t adc_read(uint8_t chan) {
	uint32_t v;
	uint8_t channels[1];

	channels[0] = chan;

	adc_set_regular_sequence(ADC1, 1, channels);
	adc_start_conversion_regular(ADC1);
	while (!(adc_eoc(ADC1)));

	v = adc_read_regular(ADC1);
	return v;
}

float sense(uint8_t chan) {
	int i;
	float s = 0.0;
	for(i=0; i<ADC_SAMPLES; i++) {
		s += (float)adc_read(chan);
	}
	return (s/ADC_SAMPLES)/4095.0;
};

float bat_sense_a() {
	return sense(1);
}

float bat_sense_v() {
	return sense(2);
}

float inp_sense_a() {
	return sense(6);
}

/* PWM */

uint8_t cur_led_mode = 0;
uint16_t cur_led_oc = 0;
int cur_led_dir = 1;

float led_lum = 1.0;

void led_set_mode(uint8_t mode) {
	if (mode != cur_led_mode) {
		cur_led_oc = 0;
		cur_led_mode = mode;
	}
}

void led_set_luminance(float l) {
	led_lum = l;
}

/* POWER */

int power_down_event;
bool started;

float cur_bat_v = 0;
float cur_bat_a = 0;
float cur_inp_a = 0;

void power_down_postpone() {
	// delay power down - 15 minutes
	power_down_delay(TIMER_EXPIRE_MIN, 0);
}

void power_down_check_postpone() {
	if (gpio_get(EN_PORT, EN_PIN)) {
		power_down_postpone();
	}

	// do not power down if charger connected
	// NOTE: will be included in EN_PORT/EN_PIN with or gate chip
	if (gpio_get(BAT_PFO_PORT, BAT_PFO_PIN)) {
		power_down_postpone();
	}
	/*
	if (!gpio_get(SHUTDOWN_PORT, SHUTDOWN_PIN)) {
		power_down_postpone();
	}
	*/
}


void charge_enable() {
	gpio_clear(CHARGE_EN_PORT, CHARGE_EN_PIN);	
}

void charge_disable() {
	gpio_set(CHARGE_EN_PORT, CHARGE_EN_PIN);
}

void boost_enable() {
	gpio_clear(BOOST_EN_PORT, BOOST_EN_PIN);	
}

void boost_disable() {
	gpio_set(BOOST_EN_PORT, BOOST_EN_PIN);	
}

void power_up() {
	charge_enable();
	boost_enable();
}

void power_down() {
	// send shutdown to raspberry pi
	//gpio_clear(SHUTDOWN_PORT, SHUTDOWN_PIN);
	// wait 20 seconds for raspberry pi to shutdown
	wait(SHUTDOWN_WAIT_SEC * 1000);

	charge_disable();
	boost_disable();

	// enter in standby mode
	standby();
}

/* IRQ */

void led_tim_isr(void)
{
	uint16_t led_oc;
	if (timer_get_flag(LED_TIMER, TIM_SR_CC1IF)) {

		/* Clear compare interrupt flag. */
		timer_clear_flag(LED_TIMER, TIM_SR_CC1IF);

		switch (cur_led_mode) {
		case LED_MODE_OFF:
			cur_led_oc = 0;
			led_oc = 0;
			break;
		case LED_MODE_SOFT_BLINK:
			if (cur_led_dir) {
				if (cur_led_oc < LED_MAX_OC) {
					cur_led_oc += PWM_INCR;
				} else {
					cur_led_dir = 0;
				}

			} else {
				if (cur_led_oc > 0) {
					cur_led_oc -= PWM_INCR;
				} else {
					cur_led_dir = 1;
				}
			}
			led_oc = cur_led_oc;

			break;
		case LED_MODE_ON:
			cur_led_oc = LED_MAX_OC + 1;
			led_oc = LED_MAX_OC + 1;
			break;
		case LED_MODE_HARD_BLINK:
			if (cur_led_oc < LED_MAX_OC / 2) {
				if ((cur_led_oc / 16) % 2) {
					led_oc = LED_MAX_OC;
				} else {
					led_oc = 0;
				}
			} else {
				led_oc = 0;
			}
			cur_led_oc++;
			if (cur_led_oc > LED_MAX_OC)
				cur_led_oc = 0;
			break;
		default:
			cur_led_oc = 0;
			led_oc = 0;
			break;
		}

		led_pwm_set_dc(PWM_TIMER_G, PWM_CHAN_G, (int)((float)led_oc * led_lum));
	} else {
		led_pwm_set_dc(PWM_TIMER_G, PWM_CHAN_G, 0);
	}
}

void rtc_isr(void)
{
	started = false;
	led_set_mode(LED_MODE_HARD_BLINK);
	wait(1000);
	power_down_event = 1;
	rtc_disable_alarm();
	exti_reset_request(EXTI17);
}

void exti0_1_isr(void)
{
	power_down_postpone();
	exti_reset_request(EXTI0);
}

void exti4_15_isr(void)
{
	started = true;
	power_down_postpone();
	exti_reset_request(EXTI5);
}


int main(void)
{

#if 0
	clock_setup();
	gpio_setup();
	adc_setup();
	power_up();
	usart_setup();
	while(1) {
		cur_bat_v = bat_sense_v();
		cur_bat_a = bat_sense_a();
		cur_inp_a = inp_sense_a(); // FIXME does not work

		__WFI();
//		wait(1000);
	}
#endif
	#if 1
	started = false;
	cur_bat_v = 0.0;
	cur_bat_a = 0.0;
	clock_setup();
	rtc_setup();
	gpio_setup();
	gpio_irq_setup();
	adc_setup();
#ifdef USB_ENABLE
	usb_setup();
#else
	usart_setup();
#endif

#ifdef I2C_CMD_ENABLE
	i2c_slave_setup(I2C_SLAVE_ADDR);
#endif

#if 1
	led_pwm_init(PWM_RCC_G, PWM_TIMER_G);
	led_pwm_setup(PWM_TIMER_G, PWM_CHAN_G, PWM_PORT_G, PWM_PIN_G, PWM_AF_G, TIM_OCM_PWM2);
	//led_set_luminance(0.5);
	led_timer_setup();

	led_set_mode(LED_MODE_OFF);
#endif
	/* enable rtc irq */
	nvic_enable_irq(NVIC_RTC_IRQ);
	nvic_set_priority(NVIC_RTC_IRQ, 1);

	/* EXTI line 17 is connected to the RTC Alarm event */
	exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI17);

	power_down_event = 0;
	power_up();
	power_down_postpone();

	while (1) {
		power_down_check_postpone();

		if (power_down_event == 1) {
			rcc_disable_rtc_clock();
			exti_disable_request(EXTI17);
			nvic_disable_irq(NVIC_RTC_IRQ);
			power_down();
		} else {
			if (!gpio_get(BAT_CHRG_PORT, BAT_CHRG_PIN)) {
				led_set_mode(LED_MODE_SOFT_BLINK);
			} else {
				led_set_mode(LED_MODE_ON);
			}
		}

		cur_bat_v = bat_sense_v();
		cur_bat_a = bat_sense_a();
		// cur_inp_a = inp_sense_a(); // FIXME does not work

//		wait(1000);
		__WFI();
	}
	#endif

	/* should not be here */
}
#endif