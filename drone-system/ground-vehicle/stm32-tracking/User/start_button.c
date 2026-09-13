#include "start_button.h"
#include "app_config.h"
#include "delay.h"

static uint8_t g_raw_state = 0U;
static uint8_t g_stable_state = 0U;
static uint32_t g_last_change = 0U;

uint8_t StartButton_IsPressed(void)
{
	return (GPIO_ReadInputDataBit(START_BUTTON_PORT,
	                              START_BUTTON_PIN) == Bit_SET) ?
	       1U : 0U;
}

void StartButton_Reset(void)
{
	g_raw_state = StartButton_IsPressed();
	g_stable_state = g_raw_state;
	g_last_change = millis();
}

void StartButton_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	gpio.GPIO_Pin = START_BUTTON_PIN;
	gpio.GPIO_Mode = GPIO_Mode_IPD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(START_BUTTON_PORT, &gpio);
	StartButton_Reset();
}

uint8_t StartButton_WasPressed(void)
{
	uint8_t raw = StartButton_IsPressed();
	uint32_t now = millis();

	if (raw != g_raw_state)
	{
		g_raw_state = raw;
		g_last_change = now;
	}

	if (raw != g_stable_state &&
	    (uint32_t)(now - g_last_change) >= START_BUTTON_DEBOUNCE_MS)
	{
		g_stable_state = raw;
		if (g_stable_state != 0U)
		{
			return 1U;
		}
	}
	return 0U;
}
