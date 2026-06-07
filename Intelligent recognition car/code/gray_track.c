#include "headfile.h"

// 上一次循迹位置，用于掉线时保持
static int last_position = 0;

// 上一次偏差，用于增量 I 和微分 D
static int prev_position = 0;

// 偏差积分（I 项累加，有 ±10000 限幅防饱和）
static int integral = 0;


// 8 路灰度传感器 GPIO 初始化（均为内部上拉输入）
// PB12~PB15, PA8, PC13~PC15
void gray_init()
{
	gpio_init(GPIO_B, Pin_12, IU);
	gpio_init(GPIO_B, Pin_13, IU);
	gpio_init(GPIO_B, Pin_14, IU);
	gpio_init(GPIO_B, Pin_15, IU);
	gpio_init(GPIO_A, Pin_8, IU);
	gpio_init(GPIO_C, Pin_13, IU);
	gpio_init(GPIO_C, Pin_14, IU);
	gpio_init(GPIO_C, Pin_15, IU);
}


// 计算当前传感器位置（加权平均法）
// 返回值为 -400~+400，0 表示线在正中间
// 8 个传感器权重分别为 -4,-3,-2,-1,+1,+2,+3,+4
// 黑线=0（传感器在线上输出低），白=1
int get_sensor_position(void)
{
	int i;
	int weighted_sum = 0;
	int sensor_count = 0;
	static const int weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4};

	for (i = 0; i < 8; i++) {
		// 取反：线上(低电平) → 1，线外(高电平) → 0
		int val = 1 - digtal(i + 1);
		weighted_sum += weights[i] * val;
		sensor_count += val;
	}

	// 完全掉线：保持上一次位置（防止乱转）
	if (sensor_count == 0) {
		return last_position;
	}

	// 加权平均 × 100（保留百分位精度）
	last_position = weighted_sum * 100 / sensor_count;
	return last_position;
}


// 循迹控制（使用 TRACK_BASE_SPEED 作为基础速度）
void track()
{
	int position = get_sensor_position();

	// 微分项 = 位置变化率
	int derivative = position - prev_position;

	// 积分项累加（带饱和限幅，防止积分风up）
	integral += position;
	if (integral > 10000) integral = 10000;
	if (integral < -10000) integral = -10000;

	// 循迹 PID 修正量（参数在 gray_track.h 中定义）
	// 除以 100 是因为 position 有 100 倍缩放
	int correction = TRACK_K * position / 100
	               + TRACK_I * integral / 100
	               + TRACK_D * derivative / 100;

	// 差速转向：左轮加速/减速，右轮反向补偿
	int left_speed  = TRACK_BASE_SPEED + correction;
	int right_speed = TRACK_BASE_SPEED - correction;

	prev_position = position;
	motor_target_set(left_speed, right_speed);
}


// 可变速度循迹（用于起跑缓加速和末端减速）
void track_with_speed(int speed)
{
	int position = get_sensor_position();
	int derivative = position - prev_position;
	integral += position;
	if (integral > 10000) integral = 10000;
	if (integral < -10000) integral = -10000;
	int correction = TRACK_K * position / 100
	               + TRACK_I * integral / 100
	               + TRACK_D * derivative / 100;

	// 以传入速度代替默认基准速度
	int left_speed  = speed + correction;
	int right_speed = speed - correction;

	prev_position = position;
	motor_target_set(left_speed, right_speed);
}


// 读取单路灰度传感器（1~8）
// 返回 1=白(高电平)，0=黑(低电平)
unsigned char digtal(unsigned char channel)
{
	u8 value = 0;
	switch (channel)
	{
		case 1:
			if (gpio_get(GPIO_B, Pin_12) == 1) value = 1;
			else value = 0;
			break;
		case 2:
			if (gpio_get(GPIO_B, Pin_13) == 1) value = 1;
			else value = 0;
			break;
		case 3:
			if (gpio_get(GPIO_B, Pin_14) == 1) value = 1;
			else value = 0;
			break;
		case 4:
			if (gpio_get(GPIO_B, Pin_15) == 1) value = 1;
			else value = 0;
			break;
		case 5:
			if (gpio_get(GPIO_A, Pin_8) == 1) value = 1;
			else value = 0;
			break;
		case 6:
			if (gpio_get(GPIO_C, Pin_13) == 1) value = 1;
			else value = 0;
			break;
		case 7:
			if (gpio_get(GPIO_C, Pin_14) == 1) value = 1;
			else value = 0;
			break;
		case 8:
			if (gpio_get(GPIO_C, Pin_15) == 1) value = 1;
			else value = 0;
			break;
	}
	return value;
}
