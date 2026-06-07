#include "tuner_bridge.h"

// 命令就绪标志（由 USART1 中断置位，主循环轮询消费）
uint8_t tuner_cmd_ready = 0;
char    tuner_cmd_buf[TUNER_CMD_BUF_SIZE];

// 上次发送 CSV 数据的时刻（system_tick 单位）
static uint32_t last_send_tick = 0;


// 按固定间隔发送 CSV 数据给 LLM-PID-Tuner
// 格式: 时间(ms), 目标速度, 实际速度, PWM输出, 误差, Kp, Ki, Kd
void tuner_bridge_send_csv(void)
{
	// 限流：间隔不少于 TUNER_SEND_INTERVAL_MS
	if (system_tick - last_send_tick < TUNER_SEND_INTERVAL_MS / 10)
		return;
	last_send_tick = system_tick;

	// 以电机A 的速度 PID 数据作为回传（LLM 只调速度 PID）
	float setpoint = motorA.target;
	float input    = motorA.now;
	float pwm      = motorA.out;
	float error    = setpoint - input;
	float p        = motorA.p;
	float i        = motorA.i;
	float d        = motorA.d;

	// printf 通过串口1 输出
	printf("%lu,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\r\n",
	       system_tick * 10,
	       setpoint,
	       input,
	       pwm,
	       error,
	       p, i, d);
}


// 处理来自 LLM-PID-Tuner 的串口命令
// 支持的命令格式:
//   SET P:x I:x D:x  — 设置 PID 参数
//   PID x x x         — 同上（简洁格式）
//   RESET             — 恢复默认 PID 参数
//   STATUS            — 查询当前 PID 状态
void tuner_bridge_process_cmd(void)
{
	if (!tuner_cmd_ready) return;

	char *cmd = tuner_cmd_buf;
	tuner_cmd_ready = 0;

	// 命令匹配：SET 或 PID 开头
	if (strncmp(cmd, "SET ", 4) == 0 || strncmp(cmd, "PID ", 4) == 0 ||
	    strncmp(cmd, "PID", 3) == 0)
	{
		float new_kp = motorA.p, new_ki = motorA.i, new_kd = motorA.d;

		// 解析 P:/I:/D: 键值对
		char *p_str = strstr(cmd, "P:");
		char *i_str = strstr(cmd, "I:");
		char *d_str = strstr(cmd, "D:");

		if (p_str) new_kp = atof(p_str + 2);
		if (i_str) new_ki = atof(i_str + 2);
		if (d_str) new_kd = atof(d_str + 2);

		// 安全限幅：拒绝明显不合理的参数
		if (new_kp > 0 && new_kp <= 100 &&
		    new_ki >= 0 && new_ki <= 50 &&
		    new_kd >= 0 && new_kd <= 50)
		{
			// 同时更新 A/B 双电机（两轮共用同一套 PID 参数）
			motorA.p = new_kp; motorA.i = new_ki; motorA.d = new_kd;
			motorB.p = new_kp; motorB.i = new_ki; motorB.d = new_kd;
			printf("# PID Updated: P=%.3f I=%.3f D=%.3f\r\n", new_kp, new_ki, new_kd);
		}
		else
		{
			printf("# ERROR: PID params rejected\r\n");
		}
	}
	else if (strncmp(cmd, "RESET", 5) == 0)
	{
		// 恢复出厂 PID 参数
		pid_init(&motorA, DELTA_PID, 14, 14, 7);
		pid_init(&motorB, DELTA_PID, 14, 14, 7);
		printf("# System Reset\r\n");
	}
	else if (strncmp(cmd, "STATUS", 6) == 0)
	{
		// 查询当前 PID 参数和运行状态
		printf("# STATUS: Kp=%.3f Ki=%.3f Kd=%.3f Target=%.2f Now=%.2f\r\n",
		       motorA.p, motorA.i, motorA.d, motorA.target, motorA.now);
	}
}
