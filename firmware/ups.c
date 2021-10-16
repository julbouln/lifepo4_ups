#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/flash.h>

#include <libopencm3/stm32/timer.h>

#include <libopencm3/stm32/crs.h>
#include <libopencm3/stm32/syscfg.h>

#include <libopencm3/cm3/nvic.h>

#include <libopencm3/cm3/systick.h>

#include "config.h"
#include "usb.h"
#include "usart.h"
#include "i2c.h"
#include "rtc.h"
#include "pwm.h"
#include "adc.h"
#include "power.h"
#include "cmd.h"

void gpio_setup(void);
void gpio_irq_setup(void);
void rcc_clock_setup_in_hsi_out_8mhz(void);
void clock_setup(void);
void led_set_mode(uint8_t mode);
void led_set_luminance(float l);

static inline __attribute__((always_inline)) void __WFI(void)
{
	__asm volatile("wfi");
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
void gpio_irq_setup()
{
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
	rcc_clock_setup_in_hsi_out_48mhz();
//	rcc_clock_setup_in_hsi_out_8mhz();
#endif

	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_RTC);
	rcc_periph_clock_enable(RCC_PWR);
	rcc_periph_clock_enable(RCC_ADC);

	/* enable rtc unlocking */
	pwr_disable_backup_domain_write_protect();
}

/* PWM */

uint8_t led_enabled;

uint8_t cur_led_mode = 0;
uint16_t cur_led_oc = 0;
int cur_led_dir = 1;

float led_lum = 1.0;

void led_set_mode(uint8_t mode)
{
	if (led_enabled)
	{
		if (mode != cur_led_mode)
		{
			cur_led_oc = 0;
			cur_led_mode = mode;
		}
	} else {
		cur_led_oc = 0;
		cur_led_mode = LED_MODE_OFF;
	}
}

void led_set_luminance(float l)
{
	led_lum = l;
}

// FIXME: does not work
void led_enable()
{
	led_enabled = 1;
}

// FIXME: does not work
void led_disable()
{
	led_enabled = 0;
}

/* POWER */

int power_down_event;
bool started;

float cur_bat_v = 0;
float cur_bat_a = 0;
float cur_inp_a = 0;

/* IRQ */

void led_tim_isr(void)
{
	uint16_t led_oc;
	if (timer_get_flag(LED_TIMER, TIM_SR_CC1IF))
	{
		/* Clear compare interrupt flag. */
		timer_clear_flag(LED_TIMER, TIM_SR_CC1IF);

		switch (cur_led_mode)
		{
		case LED_MODE_OFF:
			cur_led_oc = 0;
			led_oc = 0;
			break;
		case LED_MODE_SOFT_BLINK:
			if (cur_led_dir)
			{
				if (cur_led_oc < LED_MAX_OC)
					cur_led_oc += PWM_INCR;
				else
					cur_led_dir = 0;
			}
			else
			{
				if (cur_led_oc > 0)
					cur_led_oc -= PWM_INCR;
				else
					cur_led_dir = 1;
			}
			led_oc = cur_led_oc;

			break;
		case LED_MODE_ON:
			cur_led_oc = LED_MAX_OC + 1;
			led_oc = LED_MAX_OC + 1;
			break;
		case LED_MODE_HARD_BLINK:
			if (cur_led_oc < LED_MAX_OC / 2)
			{
				if ((cur_led_oc / 16) % 2)
					led_oc = LED_MAX_OC;
				else
					led_oc = 0;
			}
			else
			{
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
	}
	else
	{
		led_pwm_set_dc(PWM_TIMER_G, PWM_CHAN_G, 0);
	}
}

void rtc_isr(void)
{
	started = false;
	//wait(1000);
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

/* systick */

/* Called when systick fires */
void sys_tick_handler(void)
{
	power_down_check_postpone();

	if (power_down_event > 0)
	{
		// clear shutdown pin : will be shuted down in SHUTDOWN_WAIT_SEC
		gpio_clear(SHUTDOWN_PORT, SHUTDOWN_PIN);

		led_set_mode(LED_MODE_HARD_BLINK);

		if (power_down_event == SHUTDOWN_WAIT_SEC)
		{
			rcc_disable_rtc_clock();
			exti_disable_request(EXTI17);
			nvic_disable_irq(NVIC_RTC_IRQ);
			power_down();
		}
		power_down_event++;
	}
	else
	{
		if (!gpio_get(BAT_CHRG_PORT, BAT_CHRG_PIN))
			led_set_mode(LED_MODE_SOFT_BLINK);
		else
			led_set_mode(LED_MODE_ON);
	}

	cur_bat_v = bat_sense_v();
	cur_bat_a = bat_sense_a();
	cur_inp_a = inp_sense_a();
}

/*
 * Set up timer to fire every x milliseconds
 * This is a unusual usage of systick, be very careful with the 24bit range
 * of the systick counter!  You can range from 1 to 2796ms with this.
 */
static void systick_setup(int xms)
{
	/* div8 per ST, stays compatible with M3/M4 parts, well done ST */
	systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
	/* clear counter so it starts right away */
	STK_CVR = 0;

	systick_set_reload(rcc_ahb_frequency / 8 / 1000 * xms);
	systick_counter_enable();
	systick_interrupt_enable();
}

int main(void)
{
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

	led_pwm_init(PWM_RCC_G, PWM_TIMER_G);
	led_pwm_setup(PWM_TIMER_G, PWM_CHAN_G, PWM_PORT_G, PWM_PIN_G, PWM_AF_G, TIM_OCM_PWM2);
	//led_set_luminance(0.5);
	led_timer_setup();

	led_enable();
	led_set_mode(LED_MODE_OFF);

	/* enable rtc irq */
	nvic_enable_irq(NVIC_RTC_IRQ);
	nvic_set_priority(NVIC_RTC_IRQ, 1);

	/* EXTI line 17 is connected to the RTC Alarm event */
	exti_set_trigger(EXTI17, EXTI_TRIGGER_RISING);
	exti_enable_request(EXTI17);

	power_down_event = 0;
	power_up();
	power_down_postpone();

	systick_setup(1000);

	while (1)
	{
		__WFI();
	}

	/* should not be here */
}
