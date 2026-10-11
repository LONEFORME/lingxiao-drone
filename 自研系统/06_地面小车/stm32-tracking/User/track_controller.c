#include "track_controller.h"
#include "app_config.h"

static uint32_t g_marker_since = 0U;
static uint32_t g_lost_since = 0U;
static uint32_t g_curve_candidate_since = 0U;
static uint32_t g_curve_exit_since = 0U;
static int16_t g_last_error = 0;
static int16_t g_base_speed = 0;
static int16_t g_last_left_speed = 0;
static int16_t g_last_right_speed = 0;
static int8_t g_curve_candidate_sign = 0;
static uint8_t g_curve_active = 0U;
static uint8_t g_finish_armed = 0U;

static int16_t ClampSpeed(int32_t value)
{
	if (value > TRACK_MAX_ABS_SPEED)
	{
		return TRACK_MAX_ABS_SPEED;
	}
	if (value < -TRACK_MAX_ABS_SPEED)
	{
		return -TRACK_MAX_ABS_SPEED;
	}
	return (int16_t)value;
}

static int16_t SlewSpeed(int16_t current, int16_t target)
{
	int32_t difference = (int32_t)target - (int32_t)current;

	if (difference > TRACK_SPEED_SLEW_STEP)
	{
		return (int16_t)(current + TRACK_SPEED_SLEW_STEP);
	}
	if (difference < -TRACK_SPEED_SLEW_STEP)
	{
		return (int16_t)(current - TRACK_SPEED_SLEW_STEP);
	}
	return target;
}

static void SetSmoothedSpeeds(int16_t target_left,
	                          int16_t target_right,
	                          int16_t *left_speed,
	                          int16_t *right_speed)
{
	g_last_left_speed = SlewSpeed(g_last_left_speed, target_left);
	g_last_right_speed = SlewSpeed(g_last_right_speed, target_right);
	*left_speed = g_last_left_speed;
	*right_speed = g_last_right_speed;
}

static void SetImmediateStop(int16_t *left_speed, int16_t *right_speed)
{
	g_last_left_speed = 0;
	g_last_right_speed = 0;
	*left_speed = 0;
	*right_speed = 0;
}

static void ResetCurveState(void)
{
	g_curve_candidate_since = 0U;
	g_curve_exit_since = 0U;
	g_curve_candidate_sign = 0;
	g_curve_active = 0U;
}

static void UpdateCurveState(int32_t error, uint32_t now)
{
	int32_t absolute_error;
	int8_t error_sign;

	absolute_error = (error < 0) ? -error : error;
	error_sign = (error < 0) ? -1 : 1;

	if (g_curve_active == 0U)
	{
		if (absolute_error >= CURVE_ENTER_ERROR)
		{
			if (g_curve_candidate_sign != error_sign ||
			    g_curve_candidate_since == 0U)
			{
				g_curve_candidate_sign = error_sign;
				g_curve_candidate_since = now;
			}
			else if ((uint32_t)(now - g_curve_candidate_since) >=
			         CURVE_ENTER_STABLE_MS)
			{
				g_curve_active = 1U;
				g_curve_exit_since = 0U;
			}
		}
		else
		{
			g_curve_candidate_since = 0U;
			g_curve_candidate_sign = 0;
		}
		return;
	}

	if (absolute_error <= CURVE_EXIT_ERROR)
	{
		if (g_curve_exit_since == 0U)
		{
			g_curve_exit_since = now;
		}
		else if ((uint32_t)(now - g_curve_exit_since) >=
		         CURVE_EXIT_STABLE_MS)
		{
			ResetCurveState();
		}
	}
	else
	{
		g_curve_exit_since = 0U;
	}
}

void TrackController_Start(uint32_t now, int16_t initial_speed)
{
	(void)now;
	g_marker_since = 0U;
	g_lost_since = 0U;
	g_last_error = 0;
	g_last_left_speed = ClampSpeed(initial_speed);
	g_last_right_speed = ClampSpeed(initial_speed);
	g_finish_armed = 0U;
	ResetCurveState();
}

void TrackController_SetBaseSpeed(int16_t speed)
{
	g_base_speed = speed;
}

void TrackController_SetFinishArmed(uint8_t armed)
{
	g_finish_armed = (armed != 0U) ? 1U : 0U;
	if (g_finish_armed == 0U)
	{
		g_marker_since = 0U;
	}
}

uint8_t TrackController_IsCurve(void)
{
	return g_curve_active;
}

TrackResult TrackController_Update(
	const uint8_t sensor[LINE_SENSOR_COUNT],
	uint32_t now,
	int16_t *left_speed,
	int16_t *right_speed)
{
	uint8_t i;
	uint8_t black_count = 0U;
	int32_t weighted_sum = 0;
	int32_t error;
	int32_t turn;
	int32_t turn_limit;
	uint8_t crossline;
	uint8_t finish_line;

	for (i = 0U; i < LINE_SENSOR_COUNT; i++)
	{
		if (sensor[i] == LINE_BLACK_LEVEL)
		{
			black_count++;
			weighted_sum += TRACK_WEIGHTS[i];
		}
	}

	crossline = (black_count >= CROSSLINE_MIN_SENSORS) ? 1U : 0U;
	finish_line =
		(black_count >= FINISH_LINE_MIN_SENSORS) ? 1U : 0U;

	if (g_finish_armed != 0U && finish_line != 0U)
	{
		ResetCurveState();
		if (g_marker_since == 0U)
		{
			g_marker_since = now;
		}
		if ((uint32_t)(now - g_marker_since) >= FINISH_STABLE_MS)
		{
			SetImmediateStop(left_speed, right_speed);
			return TRACK_FINISHED;
		}
	}
	else
	{
		g_marker_since = 0U;
	}

	/* Drive straight across a finish-width marker while it is confirmed. */
	if (crossline != 0U)
	{
		ResetCurveState();
		g_lost_since = 0U;
		g_last_error = 0;
		SetSmoothedSpeeds(g_base_speed, g_base_speed,
		                  left_speed, right_speed);
		return TRACK_RUNNING;
	}

	if (black_count == 0U)
	{
		ResetCurveState();
		if (g_lost_since == 0U)
		{
			g_lost_since = now;
		}
		if ((uint32_t)(now - g_lost_since) >= LOST_STOP_MS)
		{
			SetImmediateStop(left_speed, right_speed);
			return TRACK_LINE_LOST;
		}

		if (g_last_error < 0)
		{
			turn = -LOST_SEARCH_TURN;
		}
		else if (g_last_error > 0)
		{
			turn = LOST_SEARCH_TURN;
		}
		else
		{
			turn = 0;
		}
		SetSmoothedSpeeds(
			ClampSpeed(LOST_SEARCH_BASE + turn),
			ClampSpeed(LOST_SEARCH_BASE - turn),
			left_speed, right_speed);
		return TRACK_RUNNING;
	}

	g_lost_since = 0U;
	error = weighted_sum / (int32_t)black_count;
	UpdateCurveState(error, now);
	turn = (TRACK_KP_NUM * error +
	        TRACK_KD_NUM * (error - g_last_error)) /
	       TRACK_GAIN_DIVISOR;

	/*
	 * Keep left+right equal to twice the requested base speed.  Limiting
	 * the turn before clamping avoids silently reducing center speed.
	 */
	turn_limit = TRACK_TURN_LIMIT;
	if ((TRACK_MAX_ABS_SPEED - g_base_speed) < turn_limit)
	{
		turn_limit = TRACK_MAX_ABS_SPEED - g_base_speed;
	}
	if ((g_base_speed - TRACK_MIN_INNER_SPEED) < turn_limit)
	{
		turn_limit = g_base_speed - TRACK_MIN_INNER_SPEED;
	}
	if (turn_limit < 0)
	{
		turn_limit = 0;
	}

	if (turn > turn_limit)
	{
		turn = turn_limit;
	}
	else if (turn < -turn_limit)
	{
		turn = -turn_limit;
	}

	g_last_error = (int16_t)error;
	SetSmoothedSpeeds(
		ClampSpeed(g_base_speed + turn),
		ClampSpeed(g_base_speed - turn),
		left_speed, right_speed);
	return TRACK_RUNNING;
}
