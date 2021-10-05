/*
 * Anemometer.h
 *
 *  Created on: Oct 4, 2021
 *      Author: reedt
 */

#ifndef SRC_ANEMOMETER_H_
#define SRC_ANEMOMETER_H_

#include <deque>
#include <cstdint>
#include "cmsis_os.h"
#include "tim.h"
#include "adc.h"

class Anemometer {
private:

	bool newVal = false;

	volatile double MPS = 0.0;

	std::deque<std::pair<uint32_t, double>> samples;

	osThreadId_t threadID;

	static void mainThread(void * arg);

	static void overflowCallback(TIM_HandleTypeDef *htim);

//	static void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

	osMutexId_t adcLock;

	osMessageQueueId_t sampleQueue;

	static osEventFlagsId_t conversionFlags;

	static void adc_cb(ADC_HandleTypeDef *hadc);

public:
	Anemometer(osMutexId_t adc);

	virtual ~Anemometer();

	double getAverageSpeed();

	double getAngle();
};

#endif /* SRC_ANEMOMETER_H_ */
