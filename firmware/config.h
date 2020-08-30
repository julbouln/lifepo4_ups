#ifndef CONFIG_H
#define CONFIG_H

#define I2C_SLAVE_ADDR 0x32

#define TIMER_EXPIRE_MIN 1
#define SHUTDOWN_WAIT_SEC 10

#define EN_PORT GPIOA
#define EN_PIN GPIO0

#define SHUTDOWN_PORT GPIOA
#define SHUTDOWN_PIN GPIO3

#define BOOST_EN_PORT GPIOB
#define BOOST_EN_PIN GPIO1

#define CHARGE_EN_PORT GPIOA
#define CHARGE_EN_PIN GPIO7

#define PWM_PORT_G  GPIOA
#define PWM_PIN_G GPIO4
#define PWM_RCC_G RCC_TIM14
#define PWM_AF_G GPIO_AF4
#define PWM_TIMER_G TIM14
#define PWM_CHAN_G TIM_OC1

#define LED_TIMER TIM2
#define LED_TIMER_RCC RCC_TIM2
#define LED_TIMER_RST RST_TIM2
#define LED_TIMER_NVIC NVIC_TIM2_IRQ
#define led_tim_isr tim2_isr

#define ADC_SAMPLES 100

#define BAT_SENSE_A_PORT GPIOA
#define BAT_SENSE_A_PIN GPIO1

#define BAT_SENSE_V_PORT GPIOA
#define BAT_SENSE_V_PIN GPIO2

#define INP_SENSE_A_PORT GPIOA
#define INP_SENSE_A_PIN GPIO6

#define BAT_CHRG_PORT GPIOA
#define BAT_CHRG_PIN GPIO14

#define BAT_PFO_PORT GPIOA
#define BAT_PFO_PIN GPIO5

#define BAT_FAULT_PORT GPIOA
#define BAT_FAULT_PIN GPIO13

#define USART_PORT GPIOA
#define USART_TX_PIN GPIO9
#define USART_RX_PIN GPIO10


#define I2C_CMD_ENABLE

//#define USB_ENABLE
#define USB_PORT GPIOA
#define USB_DM_PIN GPIO11
#define USB_DP_PIN GPIO12
#define USB_USE_INT 1

#endif