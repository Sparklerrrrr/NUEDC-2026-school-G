#include "gray_track.h"

volatile int last_track_error = 0;
volatile int lost_line_count = 0;
volatile uint8_t poker_crossing = 0;

static const int track_weights[SENSOR_COUNT] = {-7, -5, -3, -1, 1, 3, 5, 7};

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

int get_active_sensor_count(void)
{
	int count = 0;
	for(int i = 0; i < SENSOR_COUNT; i++)
	{
		if(digtal(i + 1) == 0)
			count++;
	}
	return count;
}

int get_track_error(void)
{
	int weighted_sum = 0;
	int active_count = 0;

	for(int i = 0; i < SENSOR_COUNT; i++)
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
	int base_speed = BASE_SPEED;
	int curve_speed;
	int left_speed, right_speed;
	int abs_error;

	if(error == LINE_LOST)
	{
		lost_line_count++;

		if(lost_line_count <= POKER_LOST_THRESHOLD)
		{
			motor_target_set(base_speed, base_speed);
			return;
		}

		if(lost_line_count > REAL_LOST_THRESHOLD)
		{
			motor_target_set(MIN_SPEED, MIN_SPEED);
			return;
		}

		if(last_track_error < 0)
			motor_target_set(MIN_SPEED, base_speed);
		else if(last_track_error > 0)
			motor_target_set(base_speed, MIN_SPEED);
		else
			motor_target_set(base_speed, base_speed);
		return;
	}

	poker_crossing = 0;
	lost_line_count = 0;

	abs_error = error < 0 ? -error : error;
	curve_speed = base_speed - abs_error * CURVE_DECEL_FACTOR;
	if(curve_speed < MIN_SPEED) curve_speed = MIN_SPEED;

	if(error < 0)
	{
		left_speed = curve_speed + error * 3;
		right_speed = curve_speed - error * 3;
	}
	else
	{
		left_speed = curve_speed - error * 3;
		right_speed = curve_speed + error * 3;
	}

	if(left_speed < MIN_SPEED) left_speed = MIN_SPEED;
	if(right_speed < MIN_SPEED) right_speed = MIN_SPEED;

	last_track_error = error;
	motor_target_set(left_speed, right_speed);
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
