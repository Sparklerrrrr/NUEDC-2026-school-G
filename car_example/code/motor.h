#ifndef _motor_h
#define _motor_h
#include "headfile.h"

#define ENCODER_PPR        20
#define WHEEL_DIAMETER_CM  6.5f
#define WHEEL_PERIMETER_CM (3.14159f * WHEEL_DIAMETER_CM)

void motor_init(void);
void motorA_duty(int duty);
void motorB_duty(int duty);
void encoder_init(void);
void motor_target_set(int spe1, int spe2);
void motor_control(int left_speed, int right_speed);
void motor_direct_set(int left_duty, int right_duty);
void emergency_stop(void);
float get_total_distance(void);
void reset_distance(void);

extern int Encoder_count1, Encoder_count2;
extern int speed_now;
extern uint8_t motorA_dir, motorB_dir;
extern float total_distance;

#endif
