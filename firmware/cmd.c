#include <libopencm3/stm32/gpio.h>

#include "cmd.h"
#include "gauge.h"
#include "config.h"

void cmd_parse(char *recv, uint8_t recv_len, uint8_t *send, uint8_t *send_len)
{
	switch (recv[0]) {
	case CMD_PING:
		send[0] = recv[0];
		if(recv_len > 1) {
			send[1] = recv[1];
			*send_len = 2;
		} else {
			*send_len = 0;
		}
		break;
	case CMD_STATE: { // EN | FAULT | PFO | CHRG
		uint8_t state = 0;

		// 4bit state
		state += gpio_get(BAT_CHRG_PORT, BAT_CHRG_PIN) ? 1 : 0;
		state += (gpio_get(BAT_PFO_PORT, BAT_PFO_PIN) ? 1 : 0) << 1;
		state += (gpio_get(BAT_FAULT_PORT, BAT_FAULT_PIN) ? 1 : 0) << 2;
		state += (gpio_get(EN_PORT, EN_PIN) ? 1 : 0) << 3;

		send[0] = recv[0];
		send[1] = state;

		*send_len = 2;
		}
		break;
	case CMD_BAT_V: {
		float cur_bat_state = gauge_bat_v();

		send[0] = recv[0];
		send[1] = (uint8_t)cur_bat_state;
		send[2] = (uint8_t)((cur_bat_state - (float)send[1]) * 10.0);
		send[3] = (uint8_t)((cur_bat_state - (float)send[1] - (float)send[2] / 10.0) * 100.0);

		*send_len = 4;
		}
		break;
	case CMD_BAT_A: {
		float cur_bat_state = gauge_bat_a();

		send[0] = recv[0];
		send[1] = (uint8_t)cur_bat_state;
		send[2] = (uint8_t)((cur_bat_state - (float)send[1]) * 10.0);
		send[3] = (uint8_t)((cur_bat_state - (float)send[1] - (float)send[2] / 10.0) * 100.0);

		*send_len = 4;
		}
		break;
	case CMD_INP_A: {
		float cur_inp_state = gauge_sys_a();

		send[0] = recv[0];
		send[1] = (uint8_t)cur_inp_state;
		send[2] = (uint8_t)((cur_inp_state - (float)send[1]) * 10.0);
		send[3] = (uint8_t)((cur_inp_state - (float)send[1] - (float)send[2] / 10.0) * 100.0);

		*send_len = 4;
		}
		break;
	case CMD_BAT_G: {
		float cur_bat_state = gauge_bat_v();
		
		send[0] = recv[0];
		send[1] = gauge_bat_percent(cur_bat_state);

		*send_len = 2;
		}
		break;
	case CMD_CTL: // LED MODE(2) | BOOST(1) | CHRG(1)
		send[0] = recv[0];
		if(recv_len > 1) {
			uint8_t chrg = recv[1] & 0b1;
			uint8_t boost = (recv[1] >> 1) & 0b1;
			uint8_t led_mode = (recv[1] >> 2) & 0b11;

			if(chrg)
				charge_enable();
			else
				charge_disable();

			if(boost)
				boost_enable();
			else
				boost_disable();


			send[1] = recv[1];

			*send_len = 2;
		} else {
			*send_len = 0;
		}
		break;
	default:
		*send_len = 0;
		break;
	}

}
