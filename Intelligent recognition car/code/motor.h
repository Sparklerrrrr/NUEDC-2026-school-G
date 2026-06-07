#ifndef _motor_h
#define _motor_h
#include "headfile.h"

void motor_init(void);
void motorA_duty(int duty);
void motorB_duty(int duty);
void encoder_init(void);
void indicator_init(void);
void key_init(void);
uint8_t key1_pressed(void);
uint8_t key2_pressed(void);
void buzzer_on(void);
void buzzer_off(void);
void led_on(void);
void led_off(void);

extern int Encoder_count1, Encoder_count2;
extern int speed_now;
extern uint8_t motorA_dir, motorB_dir;

#endif
