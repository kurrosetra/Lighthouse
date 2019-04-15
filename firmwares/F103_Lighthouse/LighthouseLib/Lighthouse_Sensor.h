/*
 * Lighthouse_Sensor.h
 *
 *  Created on: Apr 14, 2019
 *      Author: miftakur
 */

#ifndef LIGHTHOUSE_SENSOR_H_
#define LIGHTHOUSE_SENSOR_H_

#include "Lighthouse_OOTX.h"

typedef struct
{
	volatile uint32_t timestamp[PULSE_BUFSIZE];
	volatile uint32_t head;
	volatile uint32_t tail;
} Pulse_Buffer_t;

void LBuffer_begin(Pulse_Buffer_t *buffer);
/* 0 == no data, 1 == data available */
int LBuffer_read(Pulse_Buffer_t *buffer, uint32_t *val);
void LBuffer_write(Pulse_Buffer_t *buffer, uint32_t val);

typedef struct
{
	uint8_t instance;
	Pulse_Buffer_t icp_falling;
	Pulse_Buffer_t icp_rising;
	uint8_t icp_counter;
	/* what was the last pulse times in each direction */
	uint32_t last_falling;
	uint32_t last_rising;
	/* When did the non-skipped rotor report 0 degrees? */
	uint32_t zero_time;
	/* Which lighthouse did we compute this sweep was for? */
	uint8_t lighthouse;
	/* Which rotor was reported by the sync pulse? */
	uint8_t axis;
	/* Have we seen a sweep pulse? */
	bool got_sweep;
	/* Have we seen a sync pulse that says skip? */
	bool got_skip;
	/* Have we seen a sync pulse that says not-skipped? */
	bool got_not_skip;
	/* Measured angles from the sweep pulses */
	float raw[4];
	float angles[4];
	Lighthouse_OOTX_t ootx;
} Sensor_t;

static const float midpoints[8] = { (62.5 * CLOCKS_PER_MICROSECOND),
		(72.9 * CLOCKS_PER_MICROSECOND), (83.3 * CLOCKS_PER_MICROSECOND), (93.8
				* CLOCKS_PER_MICROSECOND), (104 * CLOCKS_PER_MICROSECOND), (115
				* CLOCKS_PER_MICROSECOND), (125 * CLOCKS_PER_MICROSECOND), (135
				* CLOCKS_PER_MICROSECOND) };

void LSensor_begin(Sensor_t *sensor, uint8_t id);
/* process a sweep pulse and return -1 if no new pulse detected */
int LSensor_sweep_pulse(Sensor_t *sensor, unsigned when, unsigned len, unsigned duty);
/* Check for any activity,
 * return the sample index if a new angle measurement is available
 */
int LSensor_poll(Sensor_t *sensor);

#endif /* LIGHTHOUSE_SENSOR_H_ */
