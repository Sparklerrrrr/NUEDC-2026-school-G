#include "headfile.h"

#define DISPLAY_PERIOD  10

static uint8_t display_counter = 0;
static uint8_t last_poker_flag = 0;
static char poker_display_buf[17];

static const char *suit_names[] = {"", "S", "H", "D", "C", "JK", "JK"};
static const char *rank_names[] = {"", "A", "2", "3", "4", "5", "6", "7", "8", "9", "10", "J", "Q", "K"};

void check_lap(void)
{
	uint8_t target_laps;

	if(current_mode != MODE_BASIC && current_mode != MODE_VISION)
		return;

	if(lap_distance >= TRACK_CIRCUMFERENCE_CM)
	{
		lap_count++;
		lap_distance = 0;

		buzzer_on();
		buzzer_end_tick = system_tick + 50;

		led_on();
		for(volatile int i = 0; i < 200000; i++);
		led_off();
		for(volatile int i = 0; i < 100000; i++);
		led_on();
		for(volatile int i = 0; i < 200000; i++);
		led_off();

		if(current_mode == MODE_BASIC)
			target_laps = LAPS_BASIC;
		else
			target_laps = LAPS_VISION;

		if(lap_count >= target_laps)
		{
			motor_direct_set(0, 0);
			current_mode = MODE_IDLE;
		}
	}
}

void update_display(void)
{
	uint32_t elapsed_ms;
	uint32_t seconds;
	uint32_t minutes;

	OLED_ShowString(1, 1, "M:");
	switch(current_mode)
	{
		case MODE_IDLE:   OLED_ShowString(1, 3, "IDLE "); break;
		case MODE_BASIC:  OLED_ShowString(1, 3, "BASIC"); break;
		case MODE_MANUAL: OLED_ShowString(1, 3, "MAN  "); break;
		case MODE_VISION: OLED_ShowString(1, 3, "VIS  "); break;
	}
	OLED_ShowString(1, 9, "L:");
	OLED_ShowNum(1, 11, lap_count, 1);

	elapsed_ms = (system_tick - run_start_tick) * 10;
	minutes = elapsed_ms / 60000;
	seconds = (elapsed_ms % 60000) / 1000;
	OLED_ShowString(2, 1, "T:");
	OLED_ShowNum(2, 3, minutes, 2);
	OLED_ShowString(2, 5, ":");
	OLED_ShowNum(2, 6, seconds, 2);

	OLED_ShowString(2, 9, "D:");
	OLED_ShowFloat(2, 11, total_distance, 3, 1);

	OLED_ShowString(3, 1, "SPD:");
	OLED_ShowNum(3, 5, speed_now, 3);

	if(poker_new_flag && !last_poker_flag)
	{
		if(poker_suit <= 6 && poker_rank <= 13)
		{
			const char *s = suit_names[poker_suit];
			const char *r = rank_names[poker_rank];
			uint8_t idx = 0;
			OLED_ShowString(4, 1, "P:");
			while(s[idx]) idx++;
			OLED_ShowString(4, 3, (char *)s);
			OLED_ShowString(4, 3 + idx, (char *)r);
		}
	}
	last_poker_flag = poker_new_flag;
	poker_new_flag = 0;
}

int main(void)
{
	motor_init();
	encoder_init();
	uart_init(UART_1, 115200, 0);

	pid_init(&motorA, DELTA_PID, 10, 10, 5);
	pid_init(&motorB, DELTA_PID, 10, 10, 5);

	OLED_Init();
	buzzer_init();
	led_init();

	tim_interrupt_ms_init(TIM_3, 10, 0);

	OLED_Clear();
	OLED_ShowString(1, 1, "Smart Car");
	OLED_ShowString(2, 1, "Waiting...");

	while(current_mode == MODE_IDLE)
	{
		led_toggle();
		for(volatile int i = 0; i < 500000; i++);
	}

	OLED_Clear();

	while(1)
	{
		switch(current_mode)
		{
			case MODE_IDLE:
				motor_direct_set(0, 0);
				break;

			case MODE_BASIC:
				track_pid();
				check_lap();
				break;

			case MODE_MANUAL:
				break;

			case MODE_VISION:
				track_pid();
				check_lap();
				break;
		}

		display_counter++;
		if(display_counter >= DISPLAY_PERIOD)
		{
			display_counter = 0;
			update_display();
		}
	}
}
