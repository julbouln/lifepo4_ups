#include "pwm.h"
#include "config.h"

/* PWM */
/* Store current PWM frequency */
static uint32_t current_timer_cnt_period;

void timer_reset(uint32_t timer_peripheral)
{
	switch (timer_peripheral) {
#if defined(TIM1_BASE)
	case TIM1:
		rcc_periph_reset_pulse(RST_TIM1);
		break;
#endif
	case TIM2:
		rcc_periph_reset_pulse(RST_TIM2);
		break;
	case TIM3:
		rcc_periph_reset_pulse(RST_TIM3);
		break;
#if defined(TIM4_BASE)
	case TIM4:
		rcc_periph_reset_pulse(RST_TIM4);
		break;
#endif
#if defined(TIM5_BASE)
	case TIM5:
		rcc_periph_reset_pulse(RST_TIM5);
		break;
#endif
	case TIM6:
		rcc_periph_reset_pulse(RST_TIM6);
		break;
	case TIM7:
		rcc_periph_reset_pulse(RST_TIM7);
		break;
#if defined(TIM8_BASE)
	case TIM8:
		rcc_periph_reset_pulse(RST_TIM8);
		break;
#endif
/* These timers are not supported in libopencm3 yet */
	case TIM14:
		rcc_periph_reset_pulse(RST_TIM14);
		break;
	}
}

void led_timer_setup() {
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(LED_TIMER_RCC);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(LED_TIMER_NVIC);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(LED_TIMER_RST);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 * (These are actually default values after reset above, so this call
	 * is strictly unnecessary, but demos the api for alternative settings)
	 */
	timer_set_mode(LED_TIMER, TIM_CR1_CKD_CK_INT,
	               TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * Please take note that the clock source for STM32 timers
	 * might not be the raw APB1/APB2 clocks.  In various conditions they
	 * are doubled.  See the Reference Manual for full details!
	 * In our case, TIM2 on APB1 is running at double frequency, so this
	 * sets the prescaler to have the timer run at 5kHz
	 */
//	timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 5000));
	timer_set_prescaler(LED_TIMER, 8);

	/* Disable preload. */
	timer_disable_preload(LED_TIMER);
	timer_continuous_mode(LED_TIMER);

	/* count full range, as we'll update compare value continuously */
//	timer_set_period(TIM2, 65535);
	timer_set_period(LED_TIMER, 8192);

	/* Set the initual output compare value for OC1. */
	timer_set_oc_value(LED_TIMER, TIM_OC1, 0);

	/* Counter enable. */
	timer_enable_counter(LED_TIMER);

	/* Enable Channel 1 compare interrupt to recalculate compare values */
	timer_enable_irq(LED_TIMER, TIM_DIER_CC1IE);
}

void led_pwm_init(enum rcc_periph_clken rcc, uint32_t timer) {
	rcc_periph_clock_enable(rcc);
	timer_reset(timer);
}

void led_pwm_setup(uint32_t timer, uint32_t chan, uint32_t port, uint8_t pin, uint8_t af, uint32_t mode) {
	gpio_mode_setup(port, GPIO_MODE_AF, GPIO_PUPD_NONE, pin);
	gpio_set_af(port, af, pin);


	/* Set the timers global mode to:
	 * - use no divider
	 * - alignment edge
	 * - count direction up
	 * */
	timer_set_mode(timer,
	               TIM_CR1_CKD_CK_INT,
	               TIM_CR1_CMS_EDGE,
	               TIM_CR1_DIR_UP);

	/* set prescaler */
	timer_set_prescaler(timer, ((rcc_apb1_frequency * 2) / TIM_CLOCK_FREQ_HZ));
	/* enable preload */
	timer_enable_preload(timer);
	/* set continuous mode */
	timer_continuous_mode(timer);
	/* set repetition counter */
	timer_set_repetition_counter(timer, 0);
	/* set period */
	current_timer_cnt_period = ((TIM_CLOCK_FREQ_HZ / TIM_DEFAULT_PWM_FREQ_HZ) - 1);
	timer_set_period(timer, current_timer_cnt_period);

	timer_disable_oc_output(timer, chan);
	/* set OC mode for each channel */
	timer_set_oc_mode(timer, chan, mode);
	timer_enable_oc_preload(timer, chan);

	/* reset OC value for each channel */
	timer_set_oc_value(timer, chan, 0);

	/* enable OC output for each channel */
	timer_enable_oc_output(timer, chan);

	led_pwm_start(timer);
}

void led_pwm_reset(uint32_t timer)
{
	timer_reset(timer);
}

/* set OC value for a channel */
void led_pwm_start(uint32_t timer)
{
	timer_generate_event(timer, TIM_EGR_UG);
	timer_enable_counter(timer);
}

void led_pwm_set_dc(uint32_t timer, uint32_t chan, uint16_t dc_value_permillage)
{
	if (dc_value_permillage <= 1024) {
		/* calculate DC timer register value */
		uint32_t dc_tmr_reg_value = (uint32_t)(((uint64_t)current_timer_cnt_period * dc_value_permillage) / 1024);
		/* update the required channel */
		timer_set_oc_value(timer, chan, dc_tmr_reg_value);
	}
}
