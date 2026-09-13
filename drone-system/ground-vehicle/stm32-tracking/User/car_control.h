#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H

#include "stm32f10x.h"

/* 初始化 I2C1，并写入 JGB37-520 的闭环控制参数。 */
uint8_t Car_Init(void);

/* 通过寄存器 0x06 设置 1~4 号电机的闭环目标速度。 */
uint8_t Car_ControlMotorRaw(int16_t motor1, int16_t motor2,
                            int16_t motor3, int16_t motor4);

/* 通过寄存器 0x07 将四路 PWM 清零，真正停止全部电机。 */
uint8_t Car_StopAll(void);

/* 按 app_config.h 中的端口和方向设置左右轮速度。 */
uint8_t Car_SetWheelSpeed(int16_t left_speed, int16_t right_speed);

/* 两轮同时前进。 */
uint8_t Car_Forward(int16_t speed);

/*
 * Read cumulative encoder counts and convert both signs so that forward
 * travel is positive for the left and right wheels.
 */
uint8_t Car_ReadWheelEncoderTotals(int32_t *left_count,
	                                int32_t *right_count);

#endif
