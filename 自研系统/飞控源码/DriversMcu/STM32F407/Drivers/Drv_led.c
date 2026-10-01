/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作�?�1?7    ：匿名�?�刄1?7
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * �?�术Q�?1?7 �?1?7190169595
 * 描述    ：LED驱动
**********************************************************************************/
#include "Drv_led.h"

void DvrLedInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(ANO_RCC_LED, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = ANO_Pin_LED1 | ANO_Pin_LED2 | ANO_Pin_LED3;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = ANO_Pin_LED4;
	GPIO_Init(ANO_GPIO_LED, &GPIO_InitStructure);

	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED1);
	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED2);
	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED3);
	GPIO_ResetBits(ANO_GPIO_LED, ANO_Pin_LED4);
}

void LED_On_Off(uint16_t leds)
{

	if (leds & LED_R_BIT)
	{
		LED1_ON;
	}
	else
	{
		LED1_OFF;
	}
	if (leds & LED_G_BIT)
	{
		LED2_ON;
	}
	else
	{
		LED2_OFF;
	}
	if (leds & LED_B_BIT)
	{
		LED3_ON;
	}
	else
	{
		LED3_OFF;
	}
	if (leds & LED_A_BIT)
	{
		LED4_ON;
	}
	else
	{
		LED4_OFF;
	}
}

//LED�?1?71ms驱动，在1ms定时�?�?里调用�?�1?7
_led_st led;
void LED_1ms_DRV() //
{
	static u16 led_cnt[LED_NUM];
	u16 led_tmp;
	for (u8 i = 0; i < LED_NUM; i++)
	{

		if (led_cnt[i] < (s16)led.brightness[i])
		{
			//ON
			led_tmp |= (1 << i);
		}
		else
		{
			//OFF
			led_tmp &= ~(1 << i);
		}

		if (++led_cnt[i] >= 20)
		{
			led_cnt[i] = 0;
		}
	}
	//
	LED_On_Off(led_tmp);
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/
