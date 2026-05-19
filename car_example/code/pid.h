#ifndef __PID_h_
#define __PID_h_
#include "headfile.h"

enum
{
	POSITION_PID = 0,
	DELTA_PID,
};

typedef struct
{
	float target;
	float now;
	float error[3];
	float p, i, d;
	float pout, dout, iout;
	float out;

	uint32_t pid_mode;

} pid_t;

typedef enum
{
	MODE_IDLE = 0,
	MODE_BASIC = 1,
	MODE_MANUAL = 2,
	MODE_VISION = 3
} ControlMode;

void pid_cal(pid_t *pid);
void pid_control(void);
void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d);
void motor_target_set(int spe1, int spe2);
void pidout_limit(pid_t *pid);
void set_current_mode(int mode);
ControlMode get_current_mode(void);

extern pid_t motorA;
extern pid_t motorB;

extern volatile float line_kp;
extern volatile float line_kd;
extern volatile ControlMode current_mode;
extern volatile uint8_t use_pid_speed;
extern volatile uint32_t system_tick;
extern volatile uint8_t lap_count;
extern volatile float lap_distance;
extern volatile uint32_t run_start_tick;

extern volatile uint8_t poker_suit;
extern volatile uint8_t poker_rank;
extern volatile uint8_t poker_new_flag;

#define TRACK_CIRCUMFERENCE_CM  314.0f
#define LAPS_BASIC              2
#define LAPS_VISION             1

#define BASE_SPEED              80
#define MIN_SPEED               30
#define CURVE_DECEL_FACTOR      5

#endif
