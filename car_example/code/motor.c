#include "motor.h"

uint8_t motorA_dir = 1;
uint8_t motorB_dir = 1;

int Encoder_count1 = 0;
int Encoder_count2 = 0;

int speed_now = 0;
float total_distance = 0;

void motor_init(void)
{
	pwm_init(TIM_2, TIM2_CH1, 1000);
	gpio_init(GPIO_A, Pin_6, OUT_PP);
	gpio_init(GPIO_A, Pin_7, OUT_PP);

	pwm_init(TIM_2, TIM2_CH2, 1000);
	gpio_init(GPIO_B, Pin_0, OUT_PP);
	gpio_init(GPIO_B, Pin_1, OUT_PP);

	motorA_duty(0);
	motorB_duty(0);
}

void motorA_duty(int duty)
{
	pwm_update(TIM_2, TIM2_CH1, duty);
	gpio_set(GPIO_A, Pin_6, motorA_dir);
	gpio_set(GPIO_A, Pin_7, !motorA_dir);
}

void motorB_duty(int duty)
{
	pwm_update(TIM_2, TIM2_CH2, duty);
	gpio_set(GPIO_B, Pin_0, motorB_dir);
	gpio_set(GPIO_B, Pin_1, !motorB_dir);
}

void encoder_init(void)
{
	exti_init(EXTI_PA2, FALLING, 1);
	gpio_init(GPIO_A, Pin_3, IU);

	exti_init(EXTI_PA4, FALLING, 1);
	gpio_init(GPIO_A, Pin_5, IU);
}

void motor_control(int left_speed, int right_speed)
{
	motor_target_set(left_speed, right_speed);
}

void motor_direct_set(int left_duty, int right_duty)
{
	if(left_duty >= 0)
	{
		motorA_dir = 1;
		motorA_duty(left_duty);
	}
	else
	{
		motorA_dir = 0;
		motorA_duty(-left_duty);
	}

	if(right_duty >= 0)
	{
		motorB_dir = 1;
		motorB_duty(right_duty);
	}
	else
	{
		motorB_dir = 0;
		motorB_duty(-right_duty);
	}
}

void emergency_stop(void)
{
	motorA_duty(0);
	motorB_duty(0);
	motorA.target = 0;
	motorB.target = 0;
	motorA.out = 0;
	motorB.out = 0;
	motorA.iout = 0;
	motorB.iout = 0;
}

float get_total_distance(void)
{
	return total_distance;
}

void reset_distance(void)
{
	total_distance = 0;
}
