/*
 * Lighthouse_Sensor.c
 *
 *  Created on: Apr 14, 2019
 *      Author: miftakur
 */

#include "Lighthouse_Sensor.h"
#include <math.h>

static uint16_t findPulseLength(uint16_t start, uint16_t end);

void LBuffer_begin(Pulse_Buffer_t *buffer)
{
	buffer->head = buffer->tail = 0;
}

int LBuffer_read(Pulse_Buffer_t *buffer, uint32_t *val)
{
	__disable_irq();

	if (buffer->head == buffer->tail) {
		__enable_irq();
		return 0;
	}

	*val = buffer->timestamp[buffer->tail++ % PULSE_BUFSIZE];
	__enable_irq();
	return 1;
}

void LBuffer_write(Pulse_Buffer_t *buffer, uint32_t val)
{
	__disable_irq();
	uint32_t i = (buffer->head + 1) % PULSE_BUFSIZE;

	if (i != buffer->tail) {
		buffer->timestamp[buffer->head] = val;
		buffer->head = i;
	}
	__enable_irq();
}

void LSensor_begin(Sensor_t *sensor, uint8_t id)
{
	sensor->instance = id;
	LBuffer_begin(&sensor->icp_falling);
	LBuffer_begin(&sensor->icp_rising);
	sensor->got_sweep = sensor->got_skip = sensor->got_not_skip = false;
	sensor->icp_counter = 0;
}

int LSensor_sweep_pulse(Sensor_t *sensor, unsigned when, unsigned len, unsigned duty)
{
	/* Sweep! The 0 degree mark is when the rotor
	 * that was not skpped sent its high pulse.
	 * midpoint of the pulse is what we'll use
	 */
	uint32_t _pulseStart = 0, _pulseEnd = 0;
	float _pulseMid = 0.0f;
	float _delta = 0.0f;
	const int ind = sensor->lighthouse * 2 + sensor->axis;

	_pulseStart = findPulseLength(sensor->zero_time, sensor->last_falling);
	_pulseEnd = findPulseLength(sensor->zero_time, when);
	_pulseMid = (float) ((_pulseStart + _pulseEnd) / 2);

	_delta = _pulseMid - (4000.0 * CLOCKS_PER_MICROSECOND);

	bool valid = !sensor->got_sweep && (sensor->lighthouse != 9)
			&& (_delta < 8000 * CLOCKS_PER_MICROSECOND);

	/* flag that we have the sweep for this one already
	 * even if it is not a valid length
	 */
	sensor->got_sweep = 1;
	sensor->got_skip = sensor->got_not_skip = 0;
	sensor->lighthouse = 9;

	if (valid) {
		/* update our angle measurement (raw and floating point) */
		sensor->raw[ind] = _delta;
//		sensor->angles[ind] = (_delta - 4000.0 * CLOCKS_PER_MICROSECOND) * M_PI
//				/ (8333 * CLOCKS_PER_MICROSECOND);
		sensor->angles[ind] = _delta * 180.0
				/ (8333 * CLOCKS_PER_MICROSECOND);

		return ind;
	}
	else
		return -1;

}

int LSensor_poll(Sensor_t *sensor)
{
	uint32_t val = 0;
	int rc = LBuffer_read(&sensor->icp_falling, &val);
	if (rc == 1) {
		/* we have a falling edge pulse, store the time stamp */
		sensor->last_falling = val;
	}
	else
		return -1;

	rc = LBuffer_read(&sensor->icp_rising, &val);
	if (rc == 1) {
		/* We have a rising edge pulse, process it */
		const uint32_t len = findPulseLength(sensor->last_falling, val);
		const uint32_t duty = findPulseLength(sensor->last_rising, val);
		sensor->last_rising = val;

		/* short pulse means sweep by the laser. */
		if (len < 15 * CLOCKS_PER_MICROSECOND)
			return LSensor_sweep_pulse(sensor, val, len, duty);
		else {
			/* this is our first non-sweep pulse,
			 * reset our parameters to wait for our next sync.
			 */
			if (sensor->got_sweep || duty > 8000 * CLOCKS_PER_MICROSECOND) {
				sensor->lighthouse = 9;  //invalid
				sensor->got_sweep = sensor->got_skip = sensor->got_not_skip = 0;
			}

			uint8_t skip = 9, rotor = 9, data = 9;
			const uint32_t window = 4 * CLOCKS_PER_MICROSECOND;
			uint8_t midpoints_counter = 0;
			for ( midpoints_counter = 0; midpoints_counter < 8; midpoints_counter++ ) {
				if ((len > midpoints[midpoints_counter] - window)
						&& (len < midpoints[midpoints_counter] + window)) {

					skip = bitRead(midpoints_counter, 2);
					data = bitRead(midpoints_counter, 1);
					rotor = bitRead(midpoints_counter, 0);
					break;
				}
			}

			if (midpoints_counter < 8) {

				if (skip == 0) {
					/* store the time of the rising edge of this pulse
					 * and the rotor that is being sent
					 */
					sensor->zero_time = sensor->last_falling;
					sensor->axis = rotor;
					sensor->got_not_skip = 1;
					/* if we have already seen the skip sync pluse,
					 * then this is lighthouse 0,
					 */
					if (sensor->got_skip) {
						sensor->lighthouse = 0;
						LOotx_add_bit(&sensor->ootx, data);
					}
				}
				else if (skip == 1) {
					sensor->got_skip = 1;
					/* if we have already seen the not-skip sync pulse,
					 * then this is lighthouse 1
					 */
					if (sensor->got_not_skip) {
						sensor->lighthouse = 1;
						LOotx_add_bit(&sensor->ootx, data);
					}
				}
			}
			else {
				/* pulse invalid detected */
				sensor->lighthouse = 9;  //invalid
				sensor->got_sweep = sensor->got_skip = sensor->got_not_skip = 0;
			}
		}
	}

	return -1;
}

static uint16_t findPulseLength(uint16_t start, uint16_t end)
{
	uint16_t ret = 0;
	if (end >= start)
		ret = end - start;
	else
		ret = 0xFFFF - start + end;

	return ret;
}

