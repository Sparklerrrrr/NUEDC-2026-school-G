#include "buzzer.h"

volatile uint8_t buzzer_active = 0;
volatile uint32_t buzzer_end_tick = 0;

void buzzer_init(void)
{
	gpio_init(BUZZER_GPIO, BUZZER_PIN, OUT_PP);
	gpio_set(BUZZER_GPIO, BUZZER_PIN, 0);
}

void buzzer_on(void)
{
	gpio_set(BUZZER_GPIO, BUZZER_PIN, 1);
	buzzer_active = 1;
}

void buzzer_off(void)
{
	gpio_set(BUZZER_GPIO, BUZZER_PIN, 0);
	buzzer_active = 0;
}

void led_init(void)
{
	gpio_init(LED_GPIO, LED_PIN, OUT_PP);
	gpio_set(LED_GPIO, LED_PIN, 0);
}

void led_on(void)
{
	gpio_set(LED_GPIO, LED_PIN, 1);
}

void led_off(void)
{
	gpio_set(LED_GPIO, LED_PIN, 0);
}

void led_toggle(void)
{
	uint8_t val = gpio_get(LED_GPIO, LED_PIN);
	gpio_set(LED_GPIO, LED_PIN, !val);
}
