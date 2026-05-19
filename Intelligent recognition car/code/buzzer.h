#ifndef __buzzer_h_
#define __buzzer_h_
#include "headfile.h"

#define BUZZER_GPIO  GPIO_B
#define BUZZER_PIN   Pin_5

#define LED_GPIO     GPIO_B
#define LED_PIN      Pin_6

void buzzer_init(void);
void buzzer_on(void);
void buzzer_off(void);
void led_init(void);
void led_on(void);
void led_off(void);
void led_toggle(void);

extern volatile uint8_t buzzer_active;
extern volatile uint32_t buzzer_end_tick;

#endif
