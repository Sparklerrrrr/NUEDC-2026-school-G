#ifndef __PID_h_
#define __PID_h_
#include "headfile.h"

// PID 工作模式枚举
enum
{
	POSITION_PID = 0,  // 位置式 PID
	DELTA_PID,         // 增量式 PID（默认，适合电机速度控制）
};

// PID 控制器结构体
typedef struct
{
	float target;       // 目标值（设定速度）
	float now;          // 当前测量值（实际速度）
	float error[3];     // 偏差历史：error[0]=e(k), [1]=e(k-1), [2]=e(k-2)
	float p, i, d;      // PID 三个增益参数
	float pout, dout, iout;  // P/I/D 各项输出分量
	float out;          // PID 总输出（PWM 占空比）

	uint32_t pid_mode;  // 工作模式：POSITION_PID 或 DELTA_PID

} pid_t;

void pid_cal(pid_t *pid);
void pid_control(void);
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void motor_target_set(int spe1, int spe2);
void pidout_limit(pid_t *pid);

extern pid_t motorA;
extern pid_t motorB;
extern pid_t angle;
extern int total_distance;
extern volatile uint32_t system_tick;
#endif
