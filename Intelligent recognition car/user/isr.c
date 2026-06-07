#include "stm32f10x.h"                  // Device header
#include "headfile.h"

// ==================== 定时器中断 ====================

// TIM2: 预留（未使用）
void TIM2_IRQHandler(void)
{
	if (TIM2->SR & 1)
	{
		TIM2->SR &= ~1;
	}
}

// TIM3: 10ms 定时中断 — 驱动 PID 控制主循环
void TIM3_IRQHandler(void)
{
	if (TIM3->SR & 1)
	{
		// 每次中断执行一次 PID 计算与电机输出更新
		pid_control();

		TIM3->SR &= ~1;
	}
}

// TIM4: 预留（未使用）
void TIM4_IRQHandler(void)
{
	if (TIM4->SR & 1)
	{
		TIM4->SR &= ~1;
	}
}


// ==================== 串口中断 ====================

// USART1: 接收 LLM-PID-Tuner 调参命令
void USART1_IRQHandler(void)
{
	if (USART1->SR & 0x20)
	{
		extern uint8_t tuner_cmd_ready;
		extern char    tuner_cmd_buf[80];
		static uint8_t cmd_len = 0;

		uint8_t data = USART1->DR;

		// 换行/回车作为命令结束符
		if (data == '\r' || data == '\n')
		{
			if (cmd_len > 0)
			{
				tuner_cmd_buf[cmd_len] = '\0';
				tuner_cmd_ready = 1;   // 通知主循环处理命令
				cmd_len = 0;
			}
		}
		else if (cmd_len < sizeof(tuner_cmd_buf) - 1)
		{
			tuner_cmd_buf[cmd_len++] = data;
		}

		USART1->SR &= ~0x20;
	}
}

// USART2: 预留（未使用）
void USART2_IRQHandler(void)
{
	if (USART2->SR & 0x20)
	{
		USART2->SR &= ~0x20;
	}
}

// USART3: 预留（未使用）
void USART3_IRQHandler(void)
{
	if (USART3->SR & 0x20)
	{
		USART3->SR &= ~0x20;
	}
}


// ==================== 外部中断（编码器） ====================

void EXTI0_IRQHandler(void) // PA0/PB0/PC0 — 预留
{
	if (EXTI->PR & (1 << 0))
	{
		EXTI->PR = 1 << 0;
	}
}

void EXTI1_IRQHandler(void) // PA1/PB1/PC1 — 预留
{
	if (EXTI->PR & (1 << 1))
	{
		EXTI->PR = 1 << 1;
	}
}

// EXTI2: 编码器A（PA2 下降沿），根据方向引脚 PA3 判定正反转
void EXTI2_IRQHandler(void)
{
	if (EXTI->PR & (1 << 2))
	{
		if (gpio_get(GPIO_A, Pin_3))
			Encoder_count1--;
		else
			Encoder_count1++;

		EXTI->PR = 1 << 2;
	}
}

// EXTI3: 预留（未使用）
void EXTI3_IRQHandler(void)
{
	if (EXTI->PR & (1 << 3))
	{
		EXTI->PR = 1 << 3;
	}
}

// EXTI4: 编码器B（PA4 下降沿），根据方向引脚 PA5 判定正反转
void EXTI4_IRQHandler(void)
{
	if (EXTI->PR & (1 << 4))
	{
		if (gpio_get(GPIO_A, Pin_5))
			Encoder_count2++;
		else
			Encoder_count2--;

		EXTI->PR = 1 << 4;
	}
}

// EXTI5~9: 共享中断线
void EXTI9_5_IRQHandler(void)
{
	if (EXTI->PR & (1 << 5))   // EXTI5 — 预留
	{
		EXTI->PR = 1 << 5;
	}

	if (EXTI->PR & (1 << 6))   // EXTI6 — 预留
	{
		EXTI->PR = 1 << 6;
	}

	// EXTI7: MPU6050/HMC5883L 数据就绪 — IMU 传感器融合
	if (EXTI->PR & (1 << 7))
	{
		// 读取原始传感器数据
		MPU6050_GetData();
		HMC5883L_GetData();

		// 陀螺仪积分角度（角速度 / 16.4 * 采样周期 5ms）
		roll_gyro += (float)gx / 16.4 * 0.005;
		pitch_gyro += (float)gy / 16.4 * 0.005;
		yaw_gyro += (float)gz / 16.4 * 0.005;

		// 加速度计反算角度
		roll_acc = atan((float)ay / az) * 57.296;
		pitch_acc = -atan((float)ax / az) * 57.296;
		yaw_acc = atan((float)ay / ax) * 57.296;

		// 磁力计反算航向角
		yaw_hmc = atan2((float)hmc_x, (float)hmc_y) * 57.296;

		// 卡尔曼融合滤波：陀螺仪 + 加速度计/磁力计
		roll_Kalman = Kalman_Filter(&KF_Roll, roll_acc, (float)gx / 16.4);
		pitch_Kalman = Kalman_Filter(&KF_Pitch, pitch_acc, (float)gy / 16.4);
		yaw_Kalman = Kalman_Filter(&KF_Yaw, yaw_hmc, (float)gz / 16.4);

		EXTI->PR = 1 << 7;
	}

	if (EXTI->PR & (1 << 8))   // EXTI8 — 预留
	{
		EXTI->PR = 1 << 8;
	}

	if (EXTI->PR & (1 << 9))   // EXTI9 — 预留
	{
		EXTI->PR = 1 << 9;
	}
}
