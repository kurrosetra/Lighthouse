/*
 * Lighthouse_Config.h
 *
 *  Created on: Apr 14, 2019
 *      Author: miftakur
 */

#ifndef LIGHTHOUSE_CONFIG_H_
#define LIGHTHOUSE_CONFIG_H_

/* TODO add define for each stm32fxx here */
#if defined(STM32F103xB) && defined(USE_HAL_DRIVER)
#include "stm32f1xx.h"
#endif

#if defined(STM32F446xx) && defined(USE_HAL_DRIVER)
#include "stm32f4xx.h"
#endif	//if STM32F446xx

#include <stdbool.h>

#define CLOCKS_PER_MICROSECOND		3
#define PULSE_BUFSIZE				100
#define OOTX_BUFSIZE				64

#define USING_ARDUINO_BITMANIPULATION	1
#if USING_ARDUINO_BITMANIPULATION
#	define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#	define bitSet(value, bit) ((value) |= (1UL << (bit)))
#	define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#	define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#endif	//if USING_ARDUINO_BITMANIPULATION

#endif /* LIGHTHOUSE_CONFIG_H_ */
