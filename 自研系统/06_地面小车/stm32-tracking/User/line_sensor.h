#ifndef __LINE_SENSOR_H
#define __LINE_SENSOR_H

#include "stm32f10x.h"
#include "app_config.h"

/* I2C2: PB10=SCL and PB11=SDA, independent from the motor driver bus. */
void LineSensor_Init(void);

/*
 * Enable periodic reads. Address 0x2E register 0x02 returns one packed
 * digital byte, as defined by the official STM32 example.
 */
void LineSensor_StartDigitalStream(void);

/*
 * Copies the newest channel 1..8 values.
 * The black-line level is configured by LINE_BLACK_LEVEL in app_config.h.
 * This call also performs a scheduled I2C read.
 */
uint8_t LineSensor_GetDigital(uint8_t values[LINE_SENSOR_COUNT],
	                          uint32_t *sequence,
	                          uint32_t *age_ms);

#endif
