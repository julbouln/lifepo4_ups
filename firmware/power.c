#include "power.h"
#include "rtc.h"
#include "config.h"

static inline __attribute__((always_inline)) void __WFI(void)
{
	__asm volatile("wfi");
}

void scb_enable_deep_sleep_mode(void)
{
	SCB_SCR |= SCB_SCR_SLEEPDEEP;
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

void power_down_postpone()
{
	// delay power down - 15 minutes
	power_down_delay(TIMER_EXPIRE_MIN, 0);
}

void power_down_check_postpone()
{
	if (gpio_get(EN_PORT, EN_PIN))
	{
		power_down_postpone();
	}

	// do not power down if charger connected
	// NOTE: will be included in EN_PORT/EN_PIN with or gate chip
	if (gpio_get(BAT_PFO_PORT, BAT_PFO_PIN))
	{
		power_down_postpone();
	}
	/*
	if (!gpio_get(SHUTDOWN_PORT, SHUTDOWN_PIN)) {
		power_down_postpone();
	}
	*/
}

void charge_enable()
{
	gpio_clear(CHARGE_EN_PORT, CHARGE_EN_PIN);
}

void charge_disable()
{
	gpio_set(CHARGE_EN_PORT, CHARGE_EN_PIN);
}

void boost_enable()
{
	gpio_clear(BOOST_EN_PORT, BOOST_EN_PIN);
}

void boost_disable()
{
	gpio_set(BOOST_EN_PORT, BOOST_EN_PIN);
}

void power_up()
{
	charge_enable();
	boost_enable();
}

void power_down()
{
	// send shutdown to raspberry pi
	//gpio_clear(SHUTDOWN_PORT, SHUTDOWN_PIN);
	// wait 20 seconds for raspberry pi to shutdown
	//wait(SHUTDOWN_WAIT_SEC * 1000);

	// needed for proper standby mode ?
	charge_enable();
	boost_enable();

	// enter in standby mode
	standby();
}

void standby()
{
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
