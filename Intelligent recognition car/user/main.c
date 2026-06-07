#include "headfile.h"

// 信号状态机：完赛提示（蜂鸣器+LED 闪烁两次）
static uint8_t signal_state = 0;
static uint16_t signal_tick = 0;

// 每 10ms 调用一次，驱动蜂鸣器/LED 的短-长-短闪烁序列
static void signal_update(void)
{
	if (signal_state == 0) return;

	signal_tick++;
	switch (signal_state)
	{
		case 1:
			// 第一声短鸣（200ms）
			buzzer_on(); led_on();
			if (signal_tick >= 20) { signal_state = 2; signal_tick = 0; }
			break;
		case 2:
			// 间隔静音（100ms）
			buzzer_off(); led_off();
			if (signal_tick >= 10) { signal_state = 3; signal_tick = 0; }
			break;
		case 3:
			// 第二声短鸣（200ms）
			buzzer_on(); led_on();
			if (signal_tick >= 20) { signal_state = 4; signal_tick = 0; }
			break;
		case 4:
			// 结束闪烁，回到空闲
			buzzer_off(); led_off();
			signal_state = 0; signal_tick = 0;
			break;
	}
}

int main(void)
{
	uint8_t lap_count = 0;      // 已完成圈数
	uint8_t total_laps = 0;     // 用户选择的总圈数
	int lap_start = 0;          // 本圈起始编码器值
	uint32_t distance_cm = 0;   // 行驶距离（厘米）
	uint32_t elapsed_ms = 0;    // 已用时间（毫秒）
	uint32_t start_tick = 0;    // 发车时刻（system_tick）

	OLED_Init();

	// --- 硬件初始化 ---
	motor_init();
	encoder_init();
	gray_init();
	indicator_init();
	key_init();

	uart_init(UART_1, 115200, 0);

	// 速度 PID 初始参数（增量式，P=14 I=14 D=7）
	pid_init(&motorA, DELTA_PID, 14, 14, 7);
	pid_init(&motorB, DELTA_PID, 14, 14, 7);

	// 10ms 定时中断，驱动 pid_control() 主循环
	tim_interrupt_ms_init(TIM_3, 10, 0);

	uart_sendstr(UART_1, "Line Tracking Mode\r\n");

	// --- 等待按键选择圈数 ---
	OLED_ShowString(1, 1, "K1:1  K2:2");
	OLED_ShowString(2, 1, "Waiting...");

	while (total_laps == 0)
	{
		if (key1_pressed())
		{
			total_laps = 1;
			OLED_ShowString(1, 1, "1 Lap     ");
			buzzer_on(); delay_ms(200); buzzer_off();
		}
		else if (key2_pressed())
		{
			total_laps = 2;
			OLED_ShowString(1, 1, "2 Laps    ");
			buzzer_on(); delay_ms(200); buzzer_off();
			delay_ms(100);
			buzzer_on(); delay_ms(200); buzzer_off();
		}
	}

	// 发车前短暂延迟，让车手放手
	delay_ms(500);

	// 记录起始状态
	lap_start = total_distance;
	start_tick = system_tick;

	// --- 主行驶循环 ---
	while (1)
	{
		// 跑完所有圈数 → 停车
		if (lap_count >= total_laps)
		{
			motor_target_set(0, 0);
			uart_sendstr(UART_1, "Finished\r\n");
			break;
		}

		// 本圈剩余编码器步数
		int remaining = LAP_ENCODER_TARGET - (total_distance - lap_start);

		uint32_t elapsed_ticks = system_tick - start_tick;
		if (elapsed_ticks < 120)
		{
			// 起步缓加速：前 1.2s 从 200 线性过渡到 TRACK_BASE_SPEED
			int ramp_speed = 200 + (TRACK_BASE_SPEED - 200) * elapsed_ticks / 120;
			track_with_speed(ramp_speed);
		}
		else if (lap_count == total_laps - 1 && remaining < 800 && remaining > 0)
		{
			// 最后一圈末端减速：最后 800 步线性减速到 40
			int decel_speed = 40 + (TRACK_BASE_SPEED - 40) * remaining / 800;
			track_with_speed(decel_speed);
		}
		else
		{
			// 正常匀速循迹
			track();
		}

		// 过线检测：当前圈行驶距离达到目标
		if (remaining <= 0)
		{
			lap_count++;
			lap_start = total_distance;
			signal_state = 1;   // 触发过线信号
			signal_tick = 0;
			uart_sendstr(UART_1, "Lap: ");
			uart_sendbyte(UART_1, '0' + lap_count);
			uart_sendstr(UART_1, "\r\n");
		}

		// LLM-PID-Tuner 通信（接收调参命令 + 发送 CSV 数据）
		tuner_bridge_process_cmd();
		tuner_bridge_send_csv();

		// 更新过线提示状态机
		signal_update();

		// --- OLED 实时显示 ---
		distance_cm = (uint32_t)total_distance * 1508 / 26000;
		elapsed_ms = (system_tick - start_tick) * 10;

		OLED_ShowString(1, 1, "Lap:");
		OLED_ShowNum(1, 5, lap_count, 1);
		OLED_ShowString(1, 7, "    ");

		OLED_ShowString(2, 1, "D:");
		OLED_ShowNum(2, 3, distance_cm / 100, 3);
		OLED_ShowChar(2, 6, '.');
		OLED_ShowNum(2, 7, distance_cm % 100, 2);
		OLED_ShowString(2, 9, "m     ");

		OLED_ShowString(3, 1, "T:");
		OLED_ShowNum(3, 3, elapsed_ms / 1000, 3);
		OLED_ShowChar(3, 6, '.');
		OLED_ShowNum(3, 7, (elapsed_ms / 100) % 10, 1);
		OLED_ShowString(3, 8, "s     ");

		delay_ms(10);
	}

	// 完赛后短暂停顿
	delay_ms(500);

	// --- 显示最终成绩 ---
	distance_cm = (uint32_t)total_distance * 1508 / 26000;
	elapsed_ms = (system_tick - start_tick) * 10;

	OLED_ShowString(1, 1, "Finished!   ");
	OLED_ShowString(2, 1, "D:");
	OLED_ShowNum(2, 3, distance_cm / 100, 3);
	OLED_ShowChar(2, 6, '.');
	OLED_ShowNum(2, 7, distance_cm % 100, 2);
	OLED_ShowString(2, 9, "m     ");
	OLED_ShowString(3, 1, "T:");
	OLED_ShowNum(3, 3, elapsed_ms / 1000, 3);
	OLED_ShowChar(3, 6, '.');
	OLED_ShowNum(3, 7, (elapsed_ms / 100) % 10, 1);
	OLED_ShowString(3, 8, "s     ");

	// 完赛后 LED 不停闪烁
	while (1)
	{
		led_on();
		delay_ms(500);
		led_off();
		delay_ms(500);
	}
}
