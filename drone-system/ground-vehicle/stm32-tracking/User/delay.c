#include "delay.h"

static volatile uint32_t g_ms_tick = 0;

void Delay_Init(void)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 1000);
}

void Delay_IncTick(void)
{
	g_ms_tick++;
}

uint32_t millis(void)
{
	return g_ms_tick;
}

void Delay_ms(uint32_t ms)
{
	uint32_t start = millis();

	while ((uint32_t)(millis() - start) < ms)
	{
	}
}
