#include "gray_track.h"

volatile int last_track_error = 0;
volatile int lost_line_count = 0;

static const int track_weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

void gray_init(void)
{
	gpio_init(GPIO_B, Pin_12, IU);
	gpio_init(GPIO_B, Pin_13, IU);
	gpio_init(GPIO_B, Pin_14, IU);
	gpio_init(GPIO_B, Pin_15, IU);
	gpio_init(GPIO_A, Pin_8,  IU);
	gpio_init(GPIO_C, Pin_13, IU);
	gpio_init(GPIO_C, Pin_14, IU);
	gpio_init(GPIO_C, Pin_15, IU);
}

int get_track_error(void)
{
	int weighted_sum = 0;
	int active_count = 0;

	for(int i = 0; i < 8; i++)
	{
		if(digtal(i + 1) == 0)
		{
			weighted_sum += track_weights[i];
			active_count++;
		}
	}

	if(active_count == 0)
		return LINE_LOST;

	return weighted_sum / active_count;
}

void track_pid(void)
{
	int error = get_track_error();
	int base_duty = 20000;
	int correction;

	if(error == LINE_LOST)
	{
		lost_line_count++;
		if(lost_line_count > 100)
		{
			motor_direct_set(8000, 8000);
			return;
		}
		if(last_track_error < 0)
			motor_direct_set(5000, 20000);
		else if(last_track_error > 0)
			motor_direct_set(20000, 5000);
		else
			motor_direct_set(base_duty, base_duty);
		return;
	}

	lost_line_count = 0;

	correction = line_kp * error + line_kd * (error - last_track_error);
	last_track_error = error;

	int left_duty = base_duty - correction;
	int right_duty = base_duty + correction;

	if(left_duty > 45000) left_duty = 45000;
	if(left_duty < 0) left_duty = 0;
	if(right_duty > 45000) right_duty = 45000;
	if(right_duty < 0) right_duty = 0;

	motor_direct_set(left_duty, right_duty);
}

void track(void)
{
	if((D4 == 0) && (D5 == 0))
	{
		motor_target_set(100, 100);
	}
	else if((D4 == 0) && (D5 != 0))
	{
		motor_target_set(100, 120);
	}
	else if((D4 != 0) && (D5 == 0))
	{
		motor_target_set(120, 100);
	}
	else if((D3 != 0) && (D4 == 0))
	{
		motor_target_set(90, 130);
	}
	else if((D5 == 0) && (D6 == 0))
	{
		motor_target_set(130, 90);
	}
	else if((D3 == 0) && (D4 != 0))
	{
		motor_target_set(90, 130);
	}
	else if((D5 != 0) && (D6 == 0))
	{
		motor_target_set(130, 90);
	}
	else if((D2 == 0) && (D3 == 0))
	{
		motor_target_set(80, 150);
	}
	else if((D6 == 0) && (D7 == 0))
	{
		motor_target_set(150, 80);
	}
	else if((D2 == 0) && (D3 != 0))
	{
		motor_target_set(80, 150);
	}
	else if((D6 != 0) && (D7 == 0))
	{
		motor_target_set(150, 80);
	}
	else if((D1 == 0) && (D2 == 0))
	{
		motor_target_set(60, 180);
	}
	else if((D7 == 0) && (D8 == 0))
	{
		motor_target_set(180, 60);
	}
	else if((D1 == 0) && (D2 != 0))
	{
		motor_target_set(40, 180);
	}
	else if((D7 != 0) && (D8 == 0))
	{
		motor_target_set(180, 40);
	}
	else
	{
		motor_target_set(100, 100);
	}
}

unsigned char digtal(unsigned char channel)
{
	u8 value = 0;
	switch(channel)
	{
		case 1:
			if(gpio_get(GPIO_B, Pin_12) == 1) value = 1;
			else value = 0;
			break;
		case 2:
			if(gpio_get(GPIO_B, Pin_13) == 1) value = 1;
			else value = 0;
			break;
		case 3:
			if(gpio_get(GPIO_B, Pin_14) == 1) value = 1;
			else value = 0;
			break;
		case 4:
			if(gpio_get(GPIO_B, Pin_15) == 1) value = 1;
			else value = 0;
			break;
		case 5:
			if(gpio_get(GPIO_A, Pin_8) == 1) value = 1;
			else value = 0;
			break;
		case 6:
			if(gpio_get(GPIO_C, Pin_13) == 1) value = 1;
			else value = 0;
			break;
		case 7:
			if(gpio_get(GPIO_C, Pin_14) == 1) value = 1;
			else value = 0;
			break;
		case 8:
			if(gpio_get(GPIO_C, Pin_15) == 1) value = 1;
			else value = 0;
			break;
	}
	return value;
}
