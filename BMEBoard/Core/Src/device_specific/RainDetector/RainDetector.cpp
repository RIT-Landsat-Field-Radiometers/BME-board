/*
 * RainDetector.cpp
 *
 *  Created on: Oct 5, 2021
 *      Author: reedt
 */

#include "RainDetector.h"


osEventFlagsId_t RainDetector::conversionFlags = nullptr;

RainDetector::RainDetector(osMutexId_t adc)
{
	// TODO Auto-generated constructor stub
	adcLock = adc;
	conversionFlags = osEventFlagsNew(nullptr);
}

RainDetector::~RainDetector()
{
	// TODO Auto-generated destructor stub
}



void RainDetector::adc_cb(ADC_HandleTypeDef *hadc)
{
	// Notify thread that conversion is complete
	osEventFlagsSet(conversionFlags, 0x01);
}


bool RainDetector::isRaining()
{
	bool retval = false;
	if(osMutexAcquire(this->adcLock, 200) == osOK)
	{
		ADC_ChannelConfTypeDef sConfig =
		{ 0 };
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
		sConfig.SingleDiff = ADC_SINGLE_ENDED;
		sConfig.OffsetNumber = ADC_OFFSET_NONE;
		sConfig.Offset = 0;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			osMutexRelease(this->adcLock);
			return retval;
		}

		HAL_ADC_RegisterCallback(&hadc1, HAL_ADC_CONVERSION_COMPLETE_CB_ID, this->adc_cb);
		osEventFlagsClear(conversionFlags, 0xFFFFFFFF);
		HAL_ADC_Start_IT(&hadc1);
		if(osEventFlagsWait(conversionFlags, 0x01, osFlagsWaitAny, 200) == 0x01)
		{
			uint16_t first = HAL_ADC_GetValue(&hadc1) & 0xFFF;

			retval = first < detectionThreshold;

		}
		osMutexRelease(this->adcLock);
	}
		return retval;
}
