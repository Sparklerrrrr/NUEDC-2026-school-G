#include "pid.h"

pid_t motorA;
pid_t motorB;

volatile float line_kp = 2000.0f;
volatile float line_kd = 800.0f;

volatile ControlMode current_mode = MODE_IDLE;
volatile uint8_t use_pid_speed = 0;

volatile uint32_t system_tick = 0;

volatile uint8_t lap_count = 0;
volatile float lap_distance = 0;
volatile uint32_t run_start_tick = 0;

volatile uint8_t poker_suit = 0;
volatile uint8_t poker_rank = 0;
volatile uint8_t poker_new_flag = 0;

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
	pid->target = 0;
	pid->now = 0;
	pid->error[0] = 0;
	pid->error[1] = 0;
	pid->error[2] = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
}

void motor_target_set(int spe1, int spe2)
{
	if(spe1 >= 0)
	{
		motorA_dir = 1;
		motorA.target = spe1;
	}
	else
	{
		motorA_dir = 0;
		motorA.target = -spe1;
	}

	if(spe2 >= 0)
	{
		motorB_dir = 1;
		motorB.target = spe2;
	}
	else
	{
		motorB_dir = 0;
		motorB.target = -spe2;
	}
}

void pid_cal(pid_t *pid)
{
	pid->error[0] = pid->target - pid->now;

	if(pid->pid_mode == DELTA_PID)
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

void pidout_limit(pid_t *pid)
{
	if(pid->out >= MAX_DUTY)
		pid->out = MAX_DUTY;
	if(pid->out <= 0)
		pid->out = 0;
}

void pid_control(void)
{
	int avg_pulse;

	if(motorA_dir) motorA.now = Encoder_count1;
	else motorA.now = -Encoder_count1;
	if(motorB_dir) motorB.now = Encoder_count2;
	else motorB.now = -Encoder_count2;

	avg_pulse = (abs(Encoder_count1) + abs(Encoder_count2)) / 2;
	total_distance += (float)avg_pulse / ENCODER_PPR * WHEEL_PERIMETER_CM;
	lap_distance += (float)avg_pulse / ENCODER_PPR * WHEEL_PERIMETER_CM;
	speed_now = avg_pulse;

	Encoder_count1 = 0;
	Encoder_count2 = 0;

	if(use_pid_speed)
	{
		pid_cal(&motorA);
		pid_cal(&motorB);
		pidout_limit(&motorA);
		pidout_limit(&motorB);
		motorA_duty((int)motorA.out);
		motorB_duty((int)motorB.out);
	}
}

void set_current_mode(int mode)
{
	current_mode = (ControlMode)mode;
	if(mode == MODE_IDLE || mode == MODE_BASIC)
	{
		lap_count = 0;
		lap_distance = 0;
		total_distance = 0;
		run_start_tick = system_tick;
		last_track_error = 0;
		lost_line_count = 0;
	}
	if(mode == MODE_IDLE)
	{
		motor_direct_set(0, 0);
	}
}

ControlMode get_current_mode(void)
{
	return current_mode;
}
