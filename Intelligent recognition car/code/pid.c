#include "headfile.h"

// 电机 PID 实例（A/B 轮独立控制，共用同一套 PID 参数）
pid_t motorA;
pid_t motorB;

// 姿态 PID 实例（预留，当前项目中未使用）
pid_t angle;

// 全局累计行驶距离（编码器脉冲数的平均值）
int total_distance = 0;

// 系统心跳计数，每 10ms 加一（在 TIM3 中断的 pid_control 中递增）
volatile uint32_t system_tick = 0;

// 串口数据可视化发送（调试用，当前注释掉）
void datavision_send()
{
	// 帧头
	uart_sendbyte(UART_1, 0x03);
	uart_sendbyte(UART_1, 0xfc);

	// 发送电机A 的目标/实际速度
	uart_sendbyte(UART_1, (uint8_t)motorA.target);
	uart_sendbyte(UART_1, (uint8_t)motorA.now);

	// 帧尾
	uart_sendbyte(UART_1, 0xfc);
	uart_sendbyte(UART_1, 0x03);
}


// 初始化 PID 控制器的模式和参数
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
}

// PWM 占空比上限（对应最大输出电压的约 50%）
#define MAX_PID_OUTPUT 40000

// 电机目标速度上限（编码器单位/10ms）
#define MAX_SPEED 600

// 编码器倍率补偿：
// GMR 编码器精度是霍尔传感器的 38 倍。使用霍尔时乘以 38，使速度值统一到 GMR 量纲。
// 如果直接使用 GMR，改为 1 即可。
#define ENCODER_RATIO 38

// 设置 AB 双电机目标速度（正值=前进，负值=后退）
void motor_target_set(int spe1, int spe2)
{
	// --- 电机A ---
	if (spe1 >= 0)
	{
		motorA_dir = 1;         // 正转
		if (spe1 > MAX_SPEED)
			motorA.target = MAX_SPEED;
		else
			motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;         // 反转
		if (-spe1 > MAX_SPEED)
			motorA.target = MAX_SPEED;
		else
			motorA.target = -spe1;
	}

	// --- 电机B ---
	if (spe2 >= 0)
	{
		motorB_dir = 1;
		if (spe2 > MAX_SPEED)
			motorB.target = MAX_SPEED;
		else
			motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		if (-spe2 > MAX_SPEED)
			motorB.target = MAX_SPEED;
		else
			motorB.target = -spe2;
	}
}


// 主 PID 控制循环（每 10ms 由 TIM3 中断调用一次）
void pid_control()
{
	system_tick++;

	// 读取编码器当前速度，根据方向取符号
	if (motorA_dir) { motorA.now = -Encoder_count1; }
	else           { motorA.now =  Encoder_count1; }

	if (motorB_dir) { motorB.now = -Encoder_count2; }
	else           { motorB.now =  Encoder_count2; }

	// 累计行驶距离（两轮编码器脉冲的平均值）
	total_distance += (abs(Encoder_count1) + abs(Encoder_count2)) / 2;

	// 应用编码器倍率补偿
	motorA.now = motorA.now * ENCODER_RATIO;
	motorB.now = motorB.now * ENCODER_RATIO;

	// 清零编码器计数，准备下一周期
	Encoder_count1 = 0;
	Encoder_count2 = 0;

	// PID 计算
	pid_cal(&motorA);
	pid_cal(&motorB);

	// 输出限幅
	pidout_limit(&motorA);
	pidout_limit(&motorB);

	// 驱动电机 PWM
	motorA_duty(motorA.out);
	motorB_duty(motorB.out);
}

// PID 核心计算（支持增量式/位置式两种模式）
void pid_cal(pid_t *pid)
{
	// 当前偏差 = 目标 - 实际
	pid->error[0] = pid->target - pid->now;

	if (pid->pid_mode == DELTA_PID)  // 增量式 PID
	{
		// P: 偏差变化量  (e(k) - e(k-1))
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		// I: 当前偏差  (e(k))
		pid->iout = pid->i * pid->error[0];
		// D: 偏差的变化率  (e(k) - 2*e(k-1) + e(k-2))
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		// 增量式输出 = 上次输出 + 本次增量
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if (pid->pid_mode == POSITION_PID)  // 位置式 PID
	{
		pid->pout = pid->p * pid->error[0];
		// 位置式 I 项会累加（积分分离等其他策略可在此基础上扩展）
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// 递推偏差历史：e(k-2) ← e(k-1) ← e(k)
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

// PID 输出限幅（防止 PWM 占空比溢出）
void pidout_limit(pid_t *pid)
{
	if (pid->out >= MAX_PID_OUTPUT)
		pid->out = MAX_PID_OUTPUT;
	if (pid->out <= 0)
		pid->out = 0;
}
