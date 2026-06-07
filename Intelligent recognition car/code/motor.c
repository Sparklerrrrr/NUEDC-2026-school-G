#include "motor.h"

// 电机方向标志：1=正转（前进），0=反转（后退）
uint8_t motorA_dir = 1;
uint8_t motorB_dir = 1;

// 编码器脉冲累计值（由 EXTI 中断更新，10ms 周期清零）
int Encoder_count1 = 0;
int Encoder_count2 = 0;

// --- 电机 PWM 初始化 ---
// TIM2_CH1 → 电机A, TIM2_CH2 → 电机B
// 频率 1kHz，两路方向控制分别用 GPIO_PA6/PA7 和 GPIO_PB0/PB1
void motor_init()
{
	pwm_init(TIM_2, TIM2_CH1, 1000);
	gpio_init(GPIO_A, Pin_6, OUT_PP);
	gpio_init(GPIO_A, Pin_7, OUT_PP);

	pwm_init(TIM_2, TIM2_CH2, 1000);
	gpio_init(GPIO_B, Pin_0, OUT_PP);
	gpio_init(GPIO_B, Pin_1, OUT_PP);
}

// 设置电机A PWM 占空比 + 方向
void motorA_duty(int duty)
{
	pwm_update(TIM_2, TIM2_CH1, duty);
	gpio_set(GPIO_A, Pin_6, !motorA_dir);  // IN1
	gpio_set(GPIO_A, Pin_7, motorA_dir);   // IN2
}

// 设置电机B PWM 占空比 + 方向
void motorB_duty(int duty)
{
	pwm_update(TIM_2, TIM2_CH2, duty);
	gpio_set(GPIO_B, Pin_0, !motorB_dir);  // IN1
	gpio_set(GPIO_B, Pin_1, motorB_dir);   // IN2
}


// --- 编码器初始化 ---
// 编码器A: PA2 下降沿触发 + PA3 方向检测
// 编码器B: PA4 下降沿触发 + PA5 方向检测
void encoder_init()
{
	exti_init(EXTI_PA2, FALLING, 0);
	gpio_init(GPIO_A, Pin_3, IU);

	exti_init(EXTI_PA4, FALLING, 0);
	gpio_init(GPIO_A, Pin_5, IU);
}


// --- 蜂鸣器/LED 指示灯初始化 ---
// PB5 = 蜂鸣器, PB6 = LED
void indicator_init()
{
	gpio_init(GPIO_B, Pin_5, OUT_PP);
	gpio_init(GPIO_B, Pin_6, OUT_PP);
	gpio_set(GPIO_B, Pin_5, 0);
	gpio_set(GPIO_B, Pin_6, 0);
}


// --- 按键初始化 ---
// PB3=KEY1, PB4=KEY2（需要禁用 JTAG 复用，映射到 SWD 仅用模式）
void key_init()
{
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	AFIO->MAPR = (AFIO->MAPR & 0xF8FFFFFF) | 0x02000000;  // 禁用JTAG，保留SWD
	gpio_init(GPIO_B, Pin_3, IU);
	gpio_init(GPIO_B, Pin_4, IU);
}

// 按键1 检测（带 20ms 消抖 + 松手检测）
uint8_t key1_pressed(void)
{
	if (gpio_get(GPIO_B, Pin_3) == 0)
	{
		delay_ms(20);                         // 消抖
		if (gpio_get(GPIO_B, Pin_3) == 0)
		{
			while (gpio_get(GPIO_B, Pin_3) == 0); // 等待松手
			return 1;
		}
	}
	return 0;
}

// 按键2 检测（同上）
uint8_t key2_pressed(void)
{
	if (gpio_get(GPIO_B, Pin_4) == 0)
	{
		delay_ms(20);
		if (gpio_get(GPIO_B, Pin_4) == 0)
		{
			while (gpio_get(GPIO_B, Pin_4) == 0);
			return 1;
		}
	}
	return 0;
}

// --- 蜂鸣器/LED 直接控制 ---
void buzzer_on(void)  { gpio_set(GPIO_B, Pin_5, 1); }
void buzzer_off(void) { gpio_set(GPIO_B, Pin_5, 0); }
void led_on(void)     { gpio_set(GPIO_B, Pin_6, 1); }
void led_off(void)    { gpio_set(GPIO_B, Pin_6, 0); }
