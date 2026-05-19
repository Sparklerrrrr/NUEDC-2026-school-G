#include "stm32f10x.h"
#include "headfile.h"

#define DEFAULT_SPEED 50

#define UART_STATE_WAIT  0
#define UART_STATE_POKER_SUIT 1
#define UART_STATE_POKER_RANK 2

static volatile uint8_t uart_rx_state = UART_STATE_WAIT;

void TIM2_IRQHandler(void)
{
	if(TIM2->SR & 1)
	{
		TIM2->SR &= ~1;
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR & 1)
	{
		system_tick++;

		pid_control();

		if(buzzer_active && system_tick >= buzzer_end_tick)
		{
			buzzer_off();
		}

		TIM3->SR &= ~1;
	}
}

void TIM4_IRQHandler(void)
{
	if(TIM4->SR & 1)
	{
		TIM4->SR &= ~1;
	}
}

void USART1_IRQHandler(void)
{
	if(USART1->SR & 0x20)
	{
		uint8_t data = USART1->DR;

		switch(uart_rx_state)
		{
			case UART_STATE_WAIT:
				switch(data)
				{
					case 'W':
						if(current_mode == MODE_VISION)
							motor_direct_set(20000, 20000);
						break;
					case 'S':
						if(current_mode == MODE_VISION)
							motor_direct_set(-20000, -20000);
						break;
					case 'A':
						if(current_mode == MODE_VISION)
							motor_direct_set(-15000, 15000);
						break;
					case 'D':
						if(current_mode == MODE_VISION)
							motor_direct_set(15000, -15000);
						break;
					case 'X':
						motor_direct_set(0, 0);
						break;
					case 'E':
						emergency_stop();
						current_mode = MODE_IDLE;
						break;
					case '0':
						set_current_mode(MODE_IDLE);
						break;
					case '1':
						set_current_mode(MODE_BASIC);
						break;
					case '2':
						set_current_mode(MODE_MANUAL);
						break;
					case '3':
						set_current_mode(MODE_VISION);
						break;
					case 'P':
						uart_rx_state = UART_STATE_POKER_SUIT;
						break;
				}
				break;

			case UART_STATE_POKER_SUIT:
				poker_suit = data;
				uart_rx_state = UART_STATE_POKER_RANK;
				break;

			case UART_STATE_POKER_RANK:
				poker_rank = data;
				poker_new_flag = 1;
				uart_rx_state = UART_STATE_WAIT;
				break;
		}

		USART1->SR &= ~0x20;
	}
}

void USART2_IRQHandler(void)
{
	if(USART2->SR & 0x20)
	{
		USART2->SR &= ~0x20;
	}
}

void USART3_IRQHandler(void)
{
	if(USART3->SR & 0x20)
	{
		USART3->SR &= ~0x20;
	}
}

void EXTI0_IRQHandler(void)
{
	if(EXTI->PR & (1 << 0))
	{
		EXTI->PR = 1 << 0;
	}
}

void EXTI1_IRQHandler(void)
{
	if(EXTI->PR & (1 << 1))
	{
		EXTI->PR = 1 << 1;
	}
}

void EXTI2_IRQHandler(void)
{
	if(EXTI->PR & (1 << 2))
	{
		if(gpio_get(GPIO_A, Pin_3))
			Encoder_count1--;
		else
			Encoder_count1++;

		EXTI->PR = 1 << 2;
	}
}

void EXTI3_IRQHandler(void)
{
	if(EXTI->PR & (1 << 3))
	{
		EXTI->PR = 1 << 3;
	}
}

void EXTI4_IRQHandler(void)
{
	if(EXTI->PR & (1 << 4))
	{
		if(gpio_get(GPIO_A, Pin_5))
			Encoder_count2++;
		else
			Encoder_count2--;

		EXTI->PR = 1 << 4;
	}
}

void EXTI9_5_IRQHandler(void)
{
	if(EXTI->PR & (1 << 5))
	{
		EXTI->PR = 1 << 5;
	}

	if(EXTI->PR & (1 << 6))
	{
		EXTI->PR = 1 << 6;
	}

	if(EXTI->PR & (1 << 7))
	{
		EXTI->PR = 1 << 7;
	}

	if(EXTI->PR & (1 << 8))
	{
		EXTI->PR = 1 << 8;
	}

	if(EXTI->PR & (1 << 9))
	{
		EXTI->PR = 1 << 9;
	}
}
