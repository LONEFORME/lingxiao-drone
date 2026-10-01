#ifndef __TRACK_CONTROLLER_H
#define __TRACK_CONTROLLER_H

#include "stm32f10x.h"
#include "line_sensor.h"

typedef enum
{
	TRACK_RUNNING = 0,
	TRACK_LINE_LOST,
	TRACK_FINISHED
} TrackResult;

/* Start tracking and ramp smoothly from the speed used to cross marker A. */
void TrackController_Start(uint32_t now, int16_t initial_speed);

/* Change the straight-line speed while preserving the same PD correction. */
void TrackController_SetBaseSpeed(int16_t speed);

/* Finish-line detection is disabled until the car is inside curve D->A. */
void TrackController_SetFinishArmed(uint8_t armed);

/* Nonzero only after a sustained, same-direction large tracking error. */
uint8_t TrackController_IsCurve(void);

/*
 * Calculates left/right wheel commands from one fresh digital sensor frame.
 * x1 is the leftmost sensor and x8 is the rightmost sensor.
 */
TrackResult TrackController_Update(
	const uint8_t sensor[LINE_SENSOR_COUNT],
	uint32_t now,
	int16_t *left_speed,
	int16_t *right_speed);

#endif
