/*
 * RainDetector.h
 *
 *  Created on: Oct 5, 2021
 *      Author: reedt
 */

#ifndef SRC_RAINDETECTOR_H_
#define SRC_RAINDETECTOR_H_

#include "cmsis_os.h"
#include "adc.h"


class RainDetector
{
private:
	const uint16_t detectionThreshold = 2000;

	osMutexId_t adcLock;

	static osEventFlagsId_t conversionFlags;

	static void adc_cb(ADC_HandleTypeDef *hadc);
public:
	RainDetector(osMutexId_t adc);
	virtual ~RainDetector();

	bool isRaining();
};

#endif /* SRC_RAINDETECTOR_H_ */
