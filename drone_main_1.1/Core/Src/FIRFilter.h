/*
 * FIRFilter.h
 *
 *  Created on: Dec 1, 2022
 *      Author: sahin
 */

#ifndef SRC_FIRFILTER_H_
#define SRC_FIRFILTER_H_

#include <stdint.h>

#define FIR_FILTER_LENGTH 50

typedef struct {
	float 	buf[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	float out;
} FIRFilter;

void FIRFilter_Init(FIRFilter *fir);
float FIRFilter_Update(FIRFilter *fir, float inp);

#endif /* SRC_FIRFILTER_H_ */
