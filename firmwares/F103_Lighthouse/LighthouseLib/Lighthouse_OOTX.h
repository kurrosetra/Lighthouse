/*
 * Lighthouse_OOTX.h
 *
 *  Created on: Apr 14, 2019
 *      Author: miftakur
 */

#ifndef LIGHTHOUSE_OOTX_H_
#define LIGHTHOUSE_OOTX_H_

#include "Lighthouse_Config.h"

typedef struct
{
	bool valid;
	/* "phase" for each rotor */
	float fcal_0_phase;
	float fcal_1_phase;
	/* "tilt" for each rotor */
	float fcal_0_tilt;
	float fcal_1_tilt;
	/* "curve" for each rotor */
	float fcal_0_curve;
	float fcal_1_curve;
	/* orientation vector */
	int8_t acc_x;
	int8_t acc_y;
	int8_t acc_z;
} Lighthouse_OOTX_Info_t;

typedef struct
{
	uint8_t waiting_for_preamble;
	uint8_t waiting_for_length;
	uint32_t accumulator;
	uint16_t accumulator_bits;
	uint16_t rx_bytes;
	uint16_t padding;
} Lighthouse_OOTX_Private_t;

typedef struct
{
	uint8_t completed;
	uint16_t length;
	uint8_t bytes[OOTX_BUFSIZE];
	Lighthouse_OOTX_Private_t private;
	Lighthouse_OOTX_Info_t info;
} Lighthouse_OOTX_t;

void LOotx_begin(Lighthouse_OOTX_t *ootx, uint8_t id);
void LOotx_reset(Lighthouse_OOTX_t *ootx);
void LOotx_add_bit(Lighthouse_OOTX_t *ootx, uint8_t bit);
void LOotx_add_word(Lighthouse_OOTX_t *ootx, uint16_t word);
void LOotx_extract_data(Lighthouse_OOTX_t *ootx);

#endif /* LIGHTHOUSE_OOTX_H_ */
