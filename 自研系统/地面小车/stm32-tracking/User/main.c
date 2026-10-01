#include "stm32f10x.h"
#include "app_config.h"
#include "car_control.h"
#include "delay.h"
#include "line_sensor.h"
#include "start_button.h"
#include "track_controller.h"

/*
 * ================================================================
 *                    实车常用参数集中调节区
 * ================================================================
 * 修改下面的数值即可，不需要再到其他 .c/.h 文件中寻找。
 * 速度单位：mm/s；距离单位：mm；时间单位：ms。
 */

/* 1. PB1 任务选择和 A/B 两套四段速度 */
const uint32_t TASK_SELECT_WINDOW_MS = 1000U; /* 第一次检测到PB1高电平后，在1秒内统计有效高电平次数 */
const uint8_t TASK_B_MIN_PULSES = 4U;         /* 1～2次选A，4次及以上选B；未定义的3次安全回退到A */

/* A任务四段速度（单位：mm/s；100 mm/s = 10 cm/s） */
const int16_t TASK_A_SPEED_A_TO_B = 50;       /* A任务：A→B速度，当前60 mm/s = 6 cm/s */
const int16_t TASK_A_SPEED_B_TO_C = 150;      /* A任务：B→C速度，当前130 mm/s = 13 cm/s */
const int16_t TASK_A_SPEED_C_TO_D = 250;      /* A任务：C→D速度，当前150 mm/s = 15 cm/s */
const int16_t TASK_A_SPEED_D_TO_FINISH = 200; /* A任务：D→终点A速度，当前130 mm/s = 13 cm/s */

/* B任务四段速度；后续实车调速时只需修改下面四个数值 */
const int16_t TASK_B_SPEED_A_TO_B = 250;       /* B任务：A→B速度，当前50 mm/s = 5 cm/s */
const int16_t TASK_B_SPEED_B_TO_C = 200;      /* B任务：B→C速度，当前110 mm/s = 11 cm/s */
const int16_t TASK_B_SPEED_C_TO_D = 50;      /* B任务：C→D速度，当前130 mm/s = 13 cm/s */
const int16_t TASK_B_SPEED_D_TO_FINISH = 180; /* B任务：D→终点A速度，当前100 mm/s = 10 cm/s */

/* 起点横线前后的速度不属于四段循迹速度，A/B任务共用 */
const int16_t START_APPROACH_SPEED = 100;     /* 按键启动后、找到第一条A点横线前的直行速度 */
const int16_t START_MARKER_SPEED = 130;       /* 传感器正在跨越第一条A点横线时的直行速度 */

/* 2. 从 A 点起算的累计距离与终点启用位置 */
const uint32_t B_POINT_DISTANCE_MM = 1500U;   /* 从A点到B点的累计里程，到达后切换B→C速度 */
const uint32_t C_POINT_DISTANCE_MM = 3856U;   /* 从A点到C点的累计里程，到达后切换C→D速度 */
const uint32_t D_POINT_DISTANCE_MM = 5356U;   /* 从A点到D点的累计里程，到达后切换D→终点速度 */
const uint32_t ODOMETRY_STOP_DISTANCE_MM = 7762U; /* 当前在7712mm总路程前100mm按编码器里程强制停车 */
const uint32_t FINISH_ARM_AFTER_D_MM = 1000U; /* 过D点再走1000mm才允许识别终点，防止粗点误停车 */
const uint32_t ODOMETRY_UPDATE_MS = 100U;     /* 每100ms读取编码器，检查分段速度和总里程停车 */
const uint32_t ODOMETRY_MAX_STEP_COUNTS = 2048U; /* 单次编码器变化超过此值视为回绕/读数异常，不计入里程 */

/* 3. 循迹 PD、转向限制与 1～8 路位置权重 */
const int16_t TRACK_MAX_ABS_SPEED = 300;      /* 单轮速度绝对上限；基准速度达到此值时无差速转向余量 */
const int16_t TRACK_TURN_LIMIT = 180;         /* 左右轮最大差速修正量，越大允许更急的转向 */
const int16_t TRACK_MIN_INNER_SPEED = 30;     /* 弯道内侧轮最低30mm/s，避免停转/反转导致顿挫 */
const int16_t TRACK_SPEED_SLEW_STEP = 15;     /* 每个10ms控制周期单轮最多变15mm/s，抑制速度突变 */
const int32_t TRACK_KP_NUM = 21;              /* 比例项分子；实际Kp=21/100=0.21，越大跟线越积极 */
const int32_t TRACK_KD_NUM = 15;              /* 微分项分子；实际Kd=15/100=0.15，越大抑制摆动越强 */
const int32_t TRACK_GAIN_DIVISOR = 100;       /* PD参数公共分母，配合上面两个整数进行定点计算 */
const int16_t TRACK_WEIGHTS[LINE_SENSOR_COUNT] =
{
	/* 1～8路从车体左侧到右侧的位置偏差权重；绝对值越大表示离中心越远。 */
	-700, -500, -300, -100, 100, 300, 500, 700
};

/* 4. 起点、终点、丢线与弯道判定 */
const uint8_t CROSSLINE_MIN_SENSORS = 6U;     /* 至少6路同时见黑，才把它当作起点横线 */
const uint8_t FINISH_LINE_MIN_SENSORS = 6U;   /* 终点启用后，至少6路同时见黑才可能停车 */
const uint32_t START_LINE_STABLE_MS = 30U;    /* 起点横线连续有效30ms后才确认，过滤瞬时干扰 */
const uint32_t START_CLEAR_STABLE_MS = 80U;   /* 离开起点横线并稳定80ms后，才进入正常循迹 */
const uint32_t FINISH_STABLE_MS = 10U;        /* 终点横线连续有效10ms后停车，避免粗点误判 */
const uint32_t LOST_STOP_MS = 600U;           /* 连续丢线超过600ms后报告丢线并停车 */
const int16_t LOST_SEARCH_BASE = 50;          /* 短时丢线搜索时的基础前进速度 */
const int16_t LOST_SEARCH_TURN = 90;          /* 短时丢线搜索时的左右轮转向差速 */
const int16_t CURVE_ENTER_ERROR = 300;        /* 循迹偏差绝对值达到300，开始尝试判定进入弯道 */
const uint32_t CURVE_ENTER_STABLE_MS = 180U;  /* 同方向大偏差持续180ms，才确认进入弯道 */
const int16_t CURVE_EXIT_ERROR = 150;         /* 偏差回落到150以内，开始尝试退出弯道 */
const uint32_t CURVE_EXIT_STABLE_MS = 250U;   /* 小偏差持续250ms，才确认已经回到直道 */

/* 5. 巡线模块读取和安装方向 */
const uint8_t LINE_BLACK_LEVEL = 0U;           /* 传感器返回0表示黑线；模块逻辑相反时改为1 */
const uint8_t LINE_SENSOR_REVERSED = 0U;       /* 1路在左侧保持0；1路装在右侧时改为1 */
const uint32_t LINE_QUERY_INTERVAL_MS = 10U;   /* 每10ms读取一次8路传感器，约100Hz */
const uint32_t SENSOR_READY_MIN_MS = 500U;     /* 上电后至少等待500ms，才允许判定传感器就绪 */
const uint8_t SENSOR_READY_MIN_FRAMES = 5U;    /* 连续收到至少5帧有效数据，才进入按键等待状态 */
const uint32_t SENSOR_FRAME_TIMEOUT_MS = 300U; /* 数据超过300ms未更新时，视为传感器通信异常 */
const uint32_t SENSOR_COMMAND_RETRY_MS = 1000U;/* 通信异常时，每1000ms重新尝试启动/读取传感器 */
const uint32_t CONTROL_PERIOD_MS = 10U;        /* 循迹控制器每10ms最多计算一次，约100Hz */

/* 6. 任务识别后的原地等待时间 */
const uint32_t START_DELAY_MS = 9000U;         /* PB1任务识别完成后原地等待9秒，再开始运动 */

/* 7. 按键和 LED 时序 */
const uint32_t START_BUTTON_DEBOUNCE_MS = 30U; /* PB1高/低电平稳定30ms后才承认变化 */
const uint32_t WAIT_LED_BLINK_MS = 250U;       /* 等待传感器时灯的半周期：250ms亮、250ms灭 */
const uint32_t SENSOR_ERROR_BLINK_MS = 100U;   /* 传感器异常时灯的半周期：100ms快速闪烁 */
const uint32_t TASK_A_LED_BLINK_MS = 100U;     /* 选中A任务后快速闪烁：100ms亮、100ms灭 */
const uint32_t TASK_B_LED_BLINK_MS = 500U;     /* 选中B任务后慢速闪烁：500ms亮、500ms灭 */
const uint32_t CURVE_LED_BLINK_MS = 500U;      /* 确认进入弯道后，灯500ms亮、500ms灭 */
const uint32_t FATAL_ERROR_BLINK_MS = 150U;    /* 电机初始化/通信致命错误时的闪灯半周期 */

typedef enum
{
	DRIVE_APPROACH_START = 0,
	DRIVE_CROSSING_START,
	DRIVE_TRACKING
} DrivePhase;

typedef enum
{
	TASK_PLAN_A = 0,
	TASK_PLAN_B
} TaskPlan;

static void LED_Set(uint8_t on);

static void LED_Init(void)
{
	GPIO_InitTypeDef gpio;

	RCC_APB2PeriphClockCmd(STATUS_LED_RCC, ENABLE);
	gpio.GPIO_Pin = STATUS_LED_PIN;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(STATUS_LED_PORT, &gpio);

	RCC_APB2PeriphClockCmd(BOARD_LED_RCC, ENABLE);
	gpio.GPIO_Pin = BOARD_LED_PIN;
	GPIO_Init(BOARD_LED_PORT, &gpio);
	LED_Set(0U);
}

static void LED_Set(uint8_t on)
{
#if STATUS_LED_ACTIVE_LOW
	if (on != 0U)
	{
		GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN);
	}
	else
	{
		GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN);
	}
#else
	if (on != 0U)
	{
		GPIO_SetBits(STATUS_LED_PORT, STATUS_LED_PIN);
	}
	else
	{
		GPIO_ResetBits(STATUS_LED_PORT, STATUS_LED_PIN);
	}
#endif

#if BOARD_LED_ACTIVE_LOW
	if (on != 0U)
	{
		GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN);
	}
	else
	{
		GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN);
	}
#else
	if (on != 0U)
	{
		GPIO_SetBits(BOARD_LED_PORT, BOARD_LED_PIN);
	}
	else
	{
		GPIO_ResetBits(BOARD_LED_PORT, BOARD_LED_PIN);
	}
#endif
}

static void StopMotorReliable(void)
{
	Car_StopAll();
	Delay_ms(20);
	Car_StopAll();
	Delay_ms(20);
	Car_StopAll();
}

static void FatalMotorError(void)
{
	StopMotorReliable();
	while (1)
	{
		LED_Set(1U);
		Delay_ms(FATAL_ERROR_BLINK_MS);
		LED_Set(0U);
		Delay_ms(FATAL_ERROR_BLINK_MS);
	}
}

static void StopAndHold(void)
{
	StopMotorReliable();
	LED_Set(0U);
	while (1)
	{
		Car_StopAll();
		Delay_ms(500);
	}
}

/*
 * Start the UART stream immediately.  There is no blind 20 second delay:
 * several fresh frames plus a short minimum settling time make the sensor
 * ready.  If the module really starts slowly, this loop naturally waits.
 */
static void WaitForSensorReady(uint8_t sensor[LINE_SENSOR_COUNT],
	                           uint32_t *sequence)
{
	uint32_t start = millis();
	uint32_t last_command;
	uint32_t last_stop = start;
	uint32_t observed_sequence = 0U;
	uint8_t ready_frames = 0U;

	LineSensor_StartDigitalStream();
	last_command = millis();

	while (1)
	{
		uint32_t now = millis();
		uint32_t age = 0U;
		uint32_t current_sequence = 0U;

		if (LineSensor_GetDigital(sensor, &current_sequence, &age) != 0U &&
		    age <= SENSOR_FRAME_TIMEOUT_MS)
		{
			if (current_sequence != observed_sequence)
			{
				observed_sequence = current_sequence;
				*sequence = current_sequence;
				if (ready_frames < SENSOR_READY_MIN_FRAMES)
				{
					ready_frames++;
				}
			}

			if ((uint32_t)(now - start) >= SENSOR_READY_MIN_MS &&
			    ready_frames >= SENSOR_READY_MIN_FRAMES)
			{
				LED_Set(1U);
				return;
			}
		}
		else
		{
			ready_frames = 0U;
		}

		if ((uint32_t)(now - last_command) >= SENSOR_COMMAND_RETRY_MS)
		{
			LineSensor_StartDigitalStream();
			last_command = now;
		}

		LED_Set((uint8_t)(((now / WAIT_LED_BLINK_MS) & 1U) == 0U));
		if ((uint32_t)(now - last_stop) >= 500U)
		{
			Car_StopAll();
			last_stop = now;
		}
	}
}

static TaskPlan WaitForTaskSelectionAndStartDelay(void)
{
	TaskPlan task_plan;
	uint8_t pulse_count = 0U;
	uint32_t select_start_ms;
	uint32_t task_led_blink_ms;
	uint32_t start_delay_ms;
	uint32_t last_stop_ms;

	/* A button held during power-up must be released before it can start. */
	while (StartButton_IsPressed() != 0U)
	{
		LED_Set(1U);
	}

	StartButton_Reset();
	LED_Set(1U);
	while (StartButton_WasPressed() == 0U)
	{
		Car_StopAll();
		Delay_ms(20);
	}

	/*
	 * The first debounced rising edge opens the one-second selection window.
	 * A held HIGH therefore counts only once. Every additional pulse must go
	 * LOW and HIGH again for at least START_BUTTON_DEBOUNCE_MS per level.
	 */
	pulse_count = 1U;
	select_start_ms = millis();
	while ((uint32_t)(millis() - select_start_ms) <
	       TASK_SELECT_WINDOW_MS)
	{
		if (StartButton_WasPressed() != 0U && pulse_count < 255U)
		{
			pulse_count++;
		}
		Car_StopAll();
		LED_Set(1U);
		Delay_ms(5);
	}

	if (pulse_count >= TASK_B_MIN_PULSES)
	{
		task_plan = TASK_PLAN_B;
		task_led_blink_ms = TASK_B_LED_BLINK_MS;
	}
	else
	{
		task_plan = TASK_PLAN_A;
		task_led_blink_ms = TASK_A_LED_BLINK_MS;
	}

	/*
	 * Keep the wheels stopped during the configurable start delay. Task A
	 * flashes quickly and task B flashes slowly to show the selected plan.
	 */
	start_delay_ms = millis();
	last_stop_ms = start_delay_ms;
	while ((uint32_t)(millis() - start_delay_ms) < START_DELAY_MS)
	{
		uint32_t now = millis();

		LED_Set((uint8_t)(
			(((uint32_t)(now - start_delay_ms) /
			   task_led_blink_ms) & 1U) == 0U));
		if ((uint32_t)(now - last_stop_ms) >= 100U)
		{
			Car_StopAll();
			last_stop_ms = now;
		}
		Delay_ms(5);
	}
	LED_Set(1U);
	return task_plan;
}

static uint8_t SensorIsCrossline(
	const uint8_t sensor[LINE_SENSOR_COUNT])
{
	uint8_t i;
	uint8_t black_count = 0U;

	for (i = 0U; i < LINE_SENSOR_COUNT; i++)
	{
		if (sensor[i] == LINE_BLACK_LEVEL)
		{
			black_count++;
		}
	}
	return (black_count >= CROSSLINE_MIN_SENSORS) ? 1U : 0U;
}

static uint32_t EncoderStepMagnitude(int32_t previous, int32_t current)
{
	int32_t step;

	/* Unsigned subtraction preserves the correct signed step across wrap. */
	step = (int32_t)((uint32_t)current - (uint32_t)previous);
	if (step < 0)
	{
		return (uint32_t)(-(step + 1)) + 1U;
	}
	return (uint32_t)step;
}

static uint32_t EncoderTravelDistanceMm(uint32_t left_travel_counts,
	                                    uint32_t right_travel_counts)
{
	uint64_t total_counts;
	float distance_mm;
	float counts_per_revolution;

	total_counts = (uint64_t)left_travel_counts +
	               (uint64_t)right_travel_counts;

	counts_per_revolution =
		(float)MOTOR_PULSE_LINE_520 *
		(float)MOTOR_REDUCTION_520 *
		(float)MOTOR_ENCODER_EDGE_MULTIPLIER;
	distance_mm =
		(float)total_counts * 3.14159265f * MOTOR_WHEEL_DIA_520 /
		(2.0f * counts_per_revolution);

	if (distance_mm <= 0.0f)
	{
		return 0U;
	}
	return (uint32_t)(distance_mm + 0.5f);
}

int main(void)
{
	uint8_t sensor[LINE_SENSOR_COUNT];
	uint32_t sequence = 0U;
	uint32_t last_sequence = 0U;
	uint32_t last_control = 0U;
	uint32_t last_sensor_command = 0U;
	uint32_t start_line_since = 0U;
	uint32_t start_clear_since = 0U;
	uint32_t last_odometry = 0U;
	int32_t encoder_previous_left = 0;
	int32_t encoder_previous_right = 0;
	int32_t encoder_now_left = 0;
	int32_t encoder_now_right = 0;
	uint32_t left_travel_counts = 0U;
	uint32_t right_travel_counts = 0U;
	int16_t left_speed = 0;
	int16_t right_speed = 0;
	uint8_t odometry_started = 0U;
	uint8_t speed_segment = 0U;
	uint8_t finish_armed = 0U;
	TaskPlan task_plan;
	int16_t selected_speed_a_to_b;
	int16_t selected_speed_b_to_c;
	int16_t selected_speed_c_to_d;
	int16_t selected_speed_d_to_finish;
	TrackResult result;
	DrivePhase phase = DRIVE_APPROACH_START;

	Delay_Init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	LED_Init();
	StartButton_Init();
	LineSensor_Init();

	if (Car_Init() != 0U)
	{
		FatalMotorError();
	}
	StopMotorReliable();

	WaitForSensorReady(sensor, &sequence);
	task_plan = WaitForTaskSelectionAndStartDelay();
	if (task_plan == TASK_PLAN_B)
	{
		selected_speed_a_to_b = TASK_B_SPEED_A_TO_B;
		selected_speed_b_to_c = TASK_B_SPEED_B_TO_C;
		selected_speed_c_to_d = TASK_B_SPEED_C_TO_D;
		selected_speed_d_to_finish = TASK_B_SPEED_D_TO_FINISH;
	}
	else
	{
		selected_speed_a_to_b = TASK_A_SPEED_A_TO_B;
		selected_speed_b_to_c = TASK_A_SPEED_B_TO_C;
		selected_speed_c_to_d = TASK_A_SPEED_C_TO_D;
		selected_speed_d_to_finish = TASK_A_SPEED_D_TO_FINISH;
	}

	TrackController_SetBaseSpeed(selected_speed_a_to_b);
	TrackController_SetFinishArmed(0U);
	last_sequence = sequence;

	while (1)
	{
		uint32_t now = millis();
		uint32_t age = 0U;
		uint8_t crossline;

		/* A second debounced press stops in every running phase. */
		if (StartButton_WasPressed() != 0U)
		{
			StopAndHold();
		}

		if (odometry_started != 0U &&
		    (uint32_t)(now - last_odometry) >= ODOMETRY_UPDATE_MS)
		{
			uint32_t distance_mm;
			uint32_t left_step_counts;
			uint32_t right_step_counts;

			last_odometry = now;
			if (Car_ReadWheelEncoderTotals(&encoder_now_left,
			                               &encoder_now_right) != 0U)
			{
				FatalMotorError();
			}
			left_step_counts = EncoderStepMagnitude(
				encoder_previous_left, encoder_now_left);
			right_step_counts = EncoderStepMagnitude(
				encoder_previous_right, encoder_now_right);
			encoder_previous_left = encoder_now_left;
			encoder_previous_right = encoder_now_right;

			if (left_step_counts <= ODOMETRY_MAX_STEP_COUNTS)
			{
				left_travel_counts += left_step_counts;
			}
			if (right_step_counts <= ODOMETRY_MAX_STEP_COUNTS)
			{
				right_travel_counts += right_step_counts;
			}

			distance_mm = EncoderTravelDistanceMm(
				left_travel_counts, right_travel_counts);

			/*
			 * Independent hard stop based on the full A-to-A route length.
			 * Keep odometry running after D and after finish-line arming so a
			 * missed or dirty finish marker cannot make the car overrun.
			 */
			if (distance_mm >= ODOMETRY_STOP_DISTANCE_MM)
			{
				StopAndHold();
			}

			if (speed_segment < 3U &&
			    distance_mm >= D_POINT_DISTANCE_MM)
			{
				speed_segment = 3U;
				TrackController_SetBaseSpeed(
					selected_speed_d_to_finish);
			}
			else if (speed_segment < 2U &&
			         distance_mm >= C_POINT_DISTANCE_MM)
			{
				speed_segment = 2U;
				TrackController_SetBaseSpeed(
					selected_speed_c_to_d);
			}
			else if (speed_segment < 1U &&
			         distance_mm >= B_POINT_DISTANCE_MM)
			{
				speed_segment = 1U;
				TrackController_SetBaseSpeed(
					selected_speed_b_to_c);
			}
			if (finish_armed == 0U &&
			    distance_mm >=
			    (D_POINT_DISTANCE_MM + FINISH_ARM_AFTER_D_MM))
			{
				finish_armed = 1U;
				TrackController_SetFinishArmed(1U);
			}
		}

		if (LineSensor_GetDigital(sensor, &sequence, &age) == 0U ||
		    age > SENSOR_FRAME_TIMEOUT_MS)
		{
			Car_StopAll();
			LED_Set((uint8_t)(((now / SENSOR_ERROR_BLINK_MS) & 1U) == 0U));

			if ((uint32_t)(now - last_sensor_command) >=
			    SENSOR_COMMAND_RETRY_MS)
			{
				LineSensor_StartDigitalStream();
				last_sensor_command = now;
			}
			Delay_ms(10);
			continue;
		}

		if (phase == DRIVE_TRACKING &&
		    TrackController_IsCurve() != 0U)
		{
			LED_Set((uint8_t)(
				((now / CURVE_LED_BLINK_MS) & 1U) == 0U));
		}
		else
		{
			LED_Set(1U);
		}

		if (sequence == last_sequence ||
		    (uint32_t)(now - last_control) < CONTROL_PERIOD_MS)
		{
			continue;
		}

		last_sequence = sequence;
		last_control = now;
		crossline = SensorIsCrossline(sensor);

		if (phase == DRIVE_APPROACH_START)
		{
			left_speed = START_APPROACH_SPEED;
			right_speed = START_APPROACH_SPEED;

			if (crossline != 0U)
			{
				if (start_line_since == 0U)
				{
					start_line_since = now;
				}
				else if ((uint32_t)(now - start_line_since) >=
				         START_LINE_STABLE_MS)
				{
					/*
					 * Use the sensor crossing A as the odometry origin.
					 * This removes the unknown front-bumper-to-sensor
					 * installation offset from the C-point estimate.
					 */
					if (Car_ReadWheelEncoderTotals(
					        &encoder_previous_left,
					        &encoder_previous_right) != 0U)
					{
						FatalMotorError();
					}
					left_travel_counts = 0U;
					right_travel_counts = 0U;
					odometry_started = 1U;
					last_odometry = now;
					phase = DRIVE_CROSSING_START;
					start_clear_since = 0U;
				}
			}
			else
			{
				start_line_since = 0U;
			}
		}
		else if (phase == DRIVE_CROSSING_START)
		{
			left_speed = START_MARKER_SPEED;
			right_speed = START_MARKER_SPEED;

			if (crossline == 0U)
			{
				if (start_clear_since == 0U)
				{
					start_clear_since = now;
				}
				else if ((uint32_t)(now - start_clear_since) >=
				         START_CLEAR_STABLE_MS)
				{
					TrackController_Start(now, left_speed);
					phase = DRIVE_TRACKING;
				}
			}
			else
			{
				start_clear_since = 0U;
			}
		}
		else
		{
			result = TrackController_Update(sensor, now,
			                                &left_speed, &right_speed);

			if (result == TRACK_FINISHED)
			{
				StopAndHold();
			}

			if (result == TRACK_LINE_LOST)
			{
				if (Car_StopAll() != 0U)
				{
					FatalMotorError();
				}
				continue;
			}
		}

		if (Car_SetWheelSpeed(left_speed, right_speed) != 0U)
		{
			FatalMotorError();
		}
	}
}
