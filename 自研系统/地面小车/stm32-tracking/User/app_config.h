#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

#include "stm32f10x.h"

/*
 * ================================================================
 *                    底层硬件配置区
 * ================================================================
 * 速度、距离、循迹和时序参数已经移到 main.c 文件最前方。
 * 本文件只保留引脚、总线地址和电机型号等硬件配置。
 */

/* -------------------- 1. 8 路数字量灰度模块 -------------------- */

#define LINE_SENSOR_COUNT              8U

/*
 * 传感器独立使用 I2C2：PB10=SCL、PB11=SDA。
 * 电机板继续使用 I2C1：PB6=SCL、PB7=SDA。
 * 读取寄存器 0x02 返回 1 字节；Bit0～Bit7 对应 1～8 路。
 */
#define LINE_I2C                       I2C2
#define LINE_I2C_SCL_PORT             GPIOB
#define LINE_I2C_SCL_PIN              GPIO_Pin_10
#define LINE_I2C_SDA_PORT             GPIOB
#define LINE_I2C_SDA_PIN              GPIO_Pin_11
#define LINE_I2C_CLOCK_HZ           100000U
#define LINE_I2C_ADDR_7BIT             0x2EU
#define LINE_I2C_DIGITAL_REG           0x02U
#define LINE_I2C_TIMEOUT_MS              20U

/* -------------------- 2. 电机驱动板和 JGB37-520 -------------------- */

#define MOTOR_I2C                    I2C1
#define MOTOR_ADDR_7BIT              0x26U
#define MOTOR_I2C_TIMEOUT_MS           50U

#define MOTOR_TYPE_520                 1U
#define MOTOR_DEADZONE_520           1900U
#define MOTOR_PULSE_LINE_520           11U
#define MOTOR_REDUCTION_520            30U
#define MOTOR_WHEEL_DIA_520          68.0f

/* 编码器采用 A/B 两相四倍频计数。 */
#define MOTOR_ENCODER_EDGE_MULTIPLIER    4U

/*
 * 当前已验证的实车映射：
 * 驱动板 M1=右轮，M2=左轮；右轮前进为负指令，左轮为正指令。
 * 若以后更换端口或接线，只修改这里。
 */
#define RIGHT_WHEEL_MOTOR_INDEX         1U
#define LEFT_WHEEL_MOTOR_INDEX          2U
#define RIGHT_WHEEL_FORWARD_SIGN       (-1)
#define LEFT_WHEEL_FORWARD_SIGN          1

/* -------------------- 3. 按键和 LED 引脚 -------------------- */

#define START_BUTTON_PORT             GPIOB
#define START_BUTTON_PIN              GPIO_Pin_1

/*
 * 外接 3.3V 状态灯：PA8 输出高电平时点亮。
 * 如果使用裸 LED，必须串联 220～330Ω 限流电阻。
 */
#define STATUS_LED_RCC                RCC_APB2Periph_GPIOA
#define STATUS_LED_PORT               GPIOA
#define STATUS_LED_PIN                GPIO_Pin_8
#define STATUS_LED_ACTIVE_LOW           0U

/* Blue Pill PC13 板载 LED，低电平点亮，与 PA8 状态灯同步。 */
#define BOARD_LED_RCC                 RCC_APB2Periph_GPIOC
#define BOARD_LED_PORT                GPIOC
#define BOARD_LED_PIN                 GPIO_Pin_13
#define BOARD_LED_ACTIVE_LOW            1U

/*
 * main.c 顶部参数的跨文件声明。只在 main.c 修改数值，
 * 这里不要重复填写数值。
 */
extern const uint32_t TASK_SELECT_WINDOW_MS;
extern const uint8_t TASK_B_MIN_PULSES;
extern const int16_t TASK_A_SPEED_A_TO_B;
extern const int16_t TASK_A_SPEED_B_TO_C;
extern const int16_t TASK_A_SPEED_C_TO_D;
extern const int16_t TASK_A_SPEED_D_TO_FINISH;
extern const int16_t TASK_B_SPEED_A_TO_B;
extern const int16_t TASK_B_SPEED_B_TO_C;
extern const int16_t TASK_B_SPEED_C_TO_D;
extern const int16_t TASK_B_SPEED_D_TO_FINISH;
extern const int16_t START_APPROACH_SPEED;
extern const int16_t START_MARKER_SPEED;
extern const uint32_t B_POINT_DISTANCE_MM;
extern const uint32_t C_POINT_DISTANCE_MM;
extern const uint32_t D_POINT_DISTANCE_MM;
extern const uint32_t ODOMETRY_STOP_DISTANCE_MM;
extern const uint32_t FINISH_ARM_AFTER_D_MM;
extern const uint32_t ODOMETRY_UPDATE_MS;
extern const int16_t TRACK_MAX_ABS_SPEED;
extern const int16_t TRACK_TURN_LIMIT;
extern const int16_t TRACK_MIN_INNER_SPEED;
extern const int16_t TRACK_SPEED_SLEW_STEP;
extern const int32_t TRACK_KP_NUM;
extern const int32_t TRACK_KD_NUM;
extern const int32_t TRACK_GAIN_DIVISOR;
extern const int16_t TRACK_WEIGHTS[LINE_SENSOR_COUNT];
extern const uint8_t CROSSLINE_MIN_SENSORS;
extern const uint8_t FINISH_LINE_MIN_SENSORS;
extern const uint32_t START_LINE_STABLE_MS;
extern const uint32_t START_CLEAR_STABLE_MS;
extern const uint32_t FINISH_STABLE_MS;
extern const uint32_t LOST_STOP_MS;
extern const int16_t LOST_SEARCH_BASE;
extern const int16_t LOST_SEARCH_TURN;
extern const int16_t CURVE_ENTER_ERROR;
extern const uint32_t CURVE_ENTER_STABLE_MS;
extern const int16_t CURVE_EXIT_ERROR;
extern const uint32_t CURVE_EXIT_STABLE_MS;
extern const uint8_t LINE_BLACK_LEVEL;
extern const uint8_t LINE_SENSOR_REVERSED;
extern const uint32_t LINE_QUERY_INTERVAL_MS;
extern const uint32_t SENSOR_READY_MIN_MS;
extern const uint8_t SENSOR_READY_MIN_FRAMES;
extern const uint32_t SENSOR_FRAME_TIMEOUT_MS;
extern const uint32_t SENSOR_COMMAND_RETRY_MS;
extern const uint32_t CONTROL_PERIOD_MS;
extern const uint32_t START_DELAY_MS;
extern const uint32_t START_BUTTON_DEBOUNCE_MS;
extern const uint32_t WAIT_LED_BLINK_MS;
extern const uint32_t SENSOR_ERROR_BLINK_MS;
extern const uint32_t TASK_A_LED_BLINK_MS;
extern const uint32_t TASK_B_LED_BLINK_MS;
extern const uint32_t CURVE_LED_BLINK_MS;
extern const uint32_t FATAL_ERROR_BLINK_MS;

#endif
