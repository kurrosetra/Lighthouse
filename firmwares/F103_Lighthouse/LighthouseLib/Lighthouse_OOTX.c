/*
 * Lighthouse_OOTX.c
 *
 *  Created on: Apr 14, 2019
 *      Author: miftakur
 */

#include "Lighthouse_OOTX.h"
#include <string.h>
#include <math.h>

static int16_t f_to_i16(int16_t *to, float from);
static float i16_to_f(int16_t n);

void LOotx_begin(Lighthouse_OOTX_t *ootx, uint8_t id)
{
	LOotx_reset(ootx);
	ootx->completed = ootx->length = 0;
}

void LOotx_reset(Lighthouse_OOTX_t *ootx)
{
	Lighthouse_OOTX_Private_t *oPrivate = &ootx->private;

	oPrivate->waiting_for_preamble = oPrivate->waiting_for_length = 0;
	oPrivate->accumulator = oPrivate->accumulator_bits = 0;
	oPrivate->rx_bytes = 0;
	memset(ootx->bytes, 0, OOTX_BUFSIZE);
}

void LOotx_add_bit(Lighthouse_OOTX_t *ootx, uint8_t bit)
{
	Lighthouse_OOTX_Private_t *oPrivate = &ootx->private;

	bit &= 0b1;

	/* add this bit to our incoming word */
	oPrivate->accumulator = (oPrivate->accumulator << 1) | bit;
	oPrivate->accumulator_bits++;

	if (oPrivate->waiting_for_preamble) {
		/* 17 zeros, followed by a 1 == 18 bits */
		if (oPrivate->accumulator_bits != 18)
			return;

		/* received preamble, start on data */
		if (oPrivate->accumulator == 0b1) {
			/* first we'll need the length */
			oPrivate->waiting_for_preamble = 0;
			oPrivate->waiting_for_length = 1;
			oPrivate->accumulator = 0;
			oPrivate->accumulator_bits = 0;
			return;
		}
		/* we've received 18 bits worth of preamble
		 * but it isnt a valid thing. hold onto the
		 * last 17 bits worth of data
		 */
		oPrivate->accumulator_bits--;
		oPrivate->accumulator &= 0x1FFFF;
		return;
	}

	/* we're receiving data!  accumulate until we get a sync bit */
	if (oPrivate->accumulator_bits < 17)
		return;

	if ((oPrivate->accumulator & 0b1) == 0) {
		/* no sync bit. go back into waiting for preamble mode */
		LOotx_reset(ootx);
		return;
	}

	/* hurrah!  the sync bit was set */
	uint16_t _word = oPrivate->accumulator >> 1;
	oPrivate->accumulator = 0;
	oPrivate->accumulator_bits = 0;

	LOotx_add_word(ootx, _word);
}

void LOotx_add_word(Lighthouse_OOTX_t *ootx, uint16_t word)
{
	Lighthouse_OOTX_Private_t *oPrivate = &ootx->private;

	if (oPrivate->waiting_for_length) {
		ootx->length = (((word >> 8) & 0xFF) | ((word & 0xFF) << 8)) + 4;
		oPrivate->padding = ootx->length & 1;
		oPrivate->waiting_for_length = 0;
		oPrivate->rx_bytes = 0;

		/* error */
		if (ootx->length > OOTX_BUFSIZE)
			LOotx_reset(ootx);

		return;
	}

	ootx->bytes[oPrivate->rx_bytes++] = (word >> 8) & 0xFF;
	ootx->bytes[oPrivate->rx_bytes++] = word & 0xFF;

	if (oPrivate->rx_bytes < ootx->length + oPrivate->padding)
		return;

	/* TODO check CRC32*/

	ootx->completed = 1;
	LOotx_extract_data(ootx);

	/* reset to wait for preamble */
	LOotx_reset(ootx);
}

void LOotx_extract_data(Lighthouse_OOTX_t *ootx)
{
	Lighthouse_OOTX_Info_t *oInfo = &ootx->info;
//	uint16_t fw_version = ((uint16_t) ootx->bytes[1] << 8) | ootx->bytes[0];

	oInfo->fcal_0_phase = i16_to_f(((int16_t) ootx->bytes[0x7] << 8) | ootx->bytes[0x6]);
	oInfo->fcal_1_phase = i16_to_f(((int16_t) ootx->bytes[0x9] << 8) | ootx->bytes[0x8]);

	oInfo->fcal_0_tilt = i16_to_f(((int16_t) ootx->bytes[0xB] << 8) | ootx->bytes[0xA]);
	oInfo->fcal_1_tilt = i16_to_f(((int16_t) ootx->bytes[0xD] << 8) | ootx->bytes[0xC]);

	oInfo->fcal_0_curve = i16_to_f(((int16_t) ootx->bytes[0x11] << 8) | ootx->bytes[0x10]);
	oInfo->fcal_1_curve = i16_to_f(((int16_t) ootx->bytes[0x13] << 8) | ootx->bytes[0x12]);

	oInfo->acc_x = ootx->bytes[0x14];
	oInfo->acc_y = ootx->bytes[0x15];
	oInfo->acc_z = ootx->bytes[0x16];

}

static int16_t f_to_i16(int16_t *to, float from)
{
	int16_t i, f;

	if (fabs(from) <= 2047.999f) {
		i = (int16_t) from;
		f = (int16_t) (fabs(from * 16)) & 15;
		*to = (i << 4) | f;

		return 1;
	}

	return 0;
}

static float i16_to_f(int16_t n)
{
	float s = 1.0f;
	if (n < 0) {
		s = -1.0f;
		n = -n;
	}
	return s * ((float) (n >> 4) + ((n & 15) / 16.0f));
}
