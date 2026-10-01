#include "line_sensor.h"
#include "app_config.h"
#include "delay.h"

static uint8_t g_digital[LINE_SENSOR_COUNT];
static uint32_t g_sequence = 0U;
static uint32_t g_frame_time = 0U;
static uint32_t g_last_query_time = 0U;
static uint8_t g_valid = 0U;
static uint8_t g_polling_enabled = 0U;

static void LineSensor_I2C_Init(void)
{
	GPIO_InitTypeDef gpio;
	I2C_InitTypeDef i2c;

	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	/* I2C2 default pins: PB10=SCL and PB11=SDA, both open-drain. */
	gpio.GPIO_Pin = LINE_I2C_SCL_PIN;
	gpio.GPIO_Mode = GPIO_Mode_AF_OD;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(LINE_I2C_SCL_PORT, &gpio);

	gpio.GPIO_Pin = LINE_I2C_SDA_PIN;
	GPIO_Init(LINE_I2C_SDA_PORT, &gpio);

	I2C_DeInit(LINE_I2C);
	I2C_StructInit(&i2c);
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00U;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c.I2C_ClockSpeed = LINE_I2C_CLOCK_HZ;
	I2C_Init(LINE_I2C, &i2c);
	I2C_Cmd(LINE_I2C, ENABLE);
}

static uint8_t LineSensor_WaitFlag(uint32_t flag, FlagStatus status)
{
	uint32_t start = millis();

	while (I2C_GetFlagStatus(LINE_I2C, flag) != status)
	{
		if ((uint32_t)(millis() - start) > LINE_I2C_TIMEOUT_MS)
		{
			return 1U;
		}
	}
	return 0U;
}

static uint8_t LineSensor_WaitEvent(uint32_t event)
{
	uint32_t start = millis();

	while (I2C_CheckEvent(LINE_I2C, event) != SUCCESS)
	{
		if (I2C_GetFlagStatus(LINE_I2C, I2C_FLAG_AF) == SET)
		{
			I2C_ClearFlag(LINE_I2C, I2C_FLAG_AF);
			return 1U;
		}
		if ((uint32_t)(millis() - start) > LINE_I2C_TIMEOUT_MS)
		{
			return 1U;
		}
	}
	return 0U;
}

static void LineSensor_RestoreBus(void)
{
	I2C_AcknowledgeConfig(LINE_I2C, ENABLE);
	I2C_NACKPositionConfig(LINE_I2C, I2C_NACKPosition_Current);
}

/*
 * Official module transaction:
 *   START, 0x2E+W, register 0x02,
 *   repeated START, 0x2E+R, one data byte, NACK, STOP.
 */
static uint8_t LineSensor_ReadPacked(uint8_t *packed)
{
	uint8_t value;

	LineSensor_RestoreBus();
	if (LineSensor_WaitFlag(I2C_FLAG_BUSY, RESET) != 0U)
	{
		return 1U;
	}

	I2C_GenerateSTART(LINE_I2C, ENABLE);
	if (LineSensor_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != 0U)
	{
		return 2U;
	}

	I2C_Send7bitAddress(LINE_I2C, LINE_I2C_ADDR_7BIT << 1,
	                    I2C_Direction_Transmitter);
	if (LineSensor_WaitEvent(
		    I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != 0U)
	{
		I2C_GenerateSTOP(LINE_I2C, ENABLE);
		return 3U;
	}

	I2C_SendData(LINE_I2C, LINE_I2C_DIGITAL_REG);
	if (LineSensor_WaitEvent(I2C_EVENT_MASTER_BYTE_TRANSMITTED) != 0U)
	{
		I2C_GenerateSTOP(LINE_I2C, ENABLE);
		return 4U;
	}

	I2C_GenerateSTART(LINE_I2C, ENABLE);
	if (LineSensor_WaitEvent(I2C_EVENT_MASTER_MODE_SELECT) != 0U)
	{
		I2C_GenerateSTOP(LINE_I2C, ENABLE);
		return 5U;
	}

	I2C_Send7bitAddress(LINE_I2C, LINE_I2C_ADDR_7BIT << 1,
	                    I2C_Direction_Receiver);
	if (LineSensor_WaitFlag(I2C_FLAG_ADDR, SET) != 0U)
	{
		I2C_GenerateSTOP(LINE_I2C, ENABLE);
		LineSensor_RestoreBus();
		return 6U;
	}

	/*
	 * STM32F1 single-byte receive sequence: disable ACK before clearing
	 * ADDR, then generate STOP immediately. Keep this short sequence
	 * atomic so a UART interrupt cannot delay the STOP.
	 */
	I2C_AcknowledgeConfig(LINE_I2C, DISABLE);
	__disable_irq();
	(void)LINE_I2C->SR2;
	I2C_GenerateSTOP(LINE_I2C, ENABLE);
	__enable_irq();

	if (LineSensor_WaitFlag(I2C_FLAG_RXNE, SET) != 0U)
	{
		LineSensor_RestoreBus();
		return 7U;
	}

	value = (uint8_t)I2C_ReceiveData(LINE_I2C);
	LineSensor_RestoreBus();
	*packed = value;
	return 0U;
}

static void LineSensor_Poll(void)
{
	uint32_t now;
	uint8_t packed;
	uint8_t i;
	uint8_t bit_index;

	if (g_polling_enabled == 0U)
	{
		return;
	}

	now = millis();
	if ((uint32_t)(now - g_last_query_time) <
	    LINE_QUERY_INTERVAL_MS)
	{
		return;
	}
	g_last_query_time = now;

	if (LineSensor_ReadPacked(&packed) != 0U)
	{
		return;
	}

	for (i = 0U; i < LINE_SENSOR_COUNT; i++)
	{
		bit_index = (LINE_SENSOR_REVERSED != 0U) ?
			(uint8_t)(LINE_SENSOR_COUNT - 1U - i) : i;
		g_digital[i] = (uint8_t)((packed >> bit_index) & 0x01U);
	}
	g_frame_time = millis();
	g_sequence++;
	g_valid = 1U;
}

void LineSensor_Init(void)
{
	uint8_t i;

	LineSensor_I2C_Init();

	for (i = 0U; i < LINE_SENSOR_COUNT; i++)
	{
		g_digital[i] = 1U;
	}
	g_sequence = 0U;
	g_frame_time = 0U;
	g_valid = 0U;
	g_polling_enabled = 0U;
	g_last_query_time = millis();
}

void LineSensor_StartDigitalStream(void)
{
	g_polling_enabled = 1U;
	g_last_query_time = millis() - LINE_QUERY_INTERVAL_MS;
}

uint8_t LineSensor_GetDigital(uint8_t values[LINE_SENSOR_COUNT],
	                          uint32_t *sequence,
	                          uint32_t *age_ms)
{
	uint8_t i;

	LineSensor_Poll();
	if (g_valid == 0U)
	{
		return 0U;
	}

	for (i = 0U; i < LINE_SENSOR_COUNT; i++)
	{
		values[i] = g_digital[i];
	}
	if (sequence != 0)
	{
		*sequence = g_sequence;
	}
	if (age_ms != 0)
	{
		*age_ms = (uint32_t)(millis() - g_frame_time);
	}
	return 1U;
}
