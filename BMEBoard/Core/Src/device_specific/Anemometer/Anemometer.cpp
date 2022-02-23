/*
 * Anemometer.cpp
 *
 *  Created on: Oct 4, 2021
 *      Author: reedt
 */

#include "Anemometer.h"

typedef struct
{
	uint32_t timestamp;
	double sample;
} sample_time;

Anemometer *_this = nullptr;
uint64_t _overflowCount = 0;
osMessageQueueId_t _sampleQueue = nullptr;
bool falling = false;


osEventFlagsId_t Anemometer::conversionFlags = nullptr;


Anemometer::Anemometer(osMutexId_t adc)
{
	// TODO Auto-generated constructor stub
	adcLock = adc;
	conversionFlags = osEventFlagsNew(nullptr);
	sampleQueue = osMessageQueueNew(20, sizeof(sample_time), nullptr);
	_sampleQueue = sampleQueue;
	osThreadAttr_t Task_attributes;
//	=
//	{ .name = "AnemometerTask", .stack_size = 1024 * 4, .priority =
//			(osPriority_t) osPriorityNormal };
	Task_attributes.name = "AnemometerTask";
	Task_attributes.stack_size = 1024 * 4;
	Task_attributes.priority = (osPriority_t) osPriorityNormal;

	threadID = osThreadNew(mainThread, (void*) this, &Task_attributes);

}

void Anemometer::overflowCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM2)
	{
		// Capture timer overflow
		_overflowCount++;
	}
}

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Wind_SPD_Pin)
	{
		if (HAL_GPIO_ReadPin(Wind_SPD_GPIO_Port, Wind_SPD_Pin) == GPIO_PIN_SET)
		{
			// rising
			__HAL_TIM_SET_COUNTER(&htim2, 0);
			_overflowCount = 0;
			falling = true;
		}
		else
		{
			// falling
			double diff = __HAL_TIM_GET_COUNTER(&htim2);
			diff += _overflowCount * 4294967295;

			double period = (diff / 1000.0) / 1000.0;

			double mps = 1.00584 / period;

			sample_time next;
			next.timestamp = HAL_GetTick();
			next.sample = mps;
			falling = false;

			if (_sampleQueue != nullptr)
			{
				osMessageQueuePut(_sampleQueue, &next, 0, 0);
			}
		}
	}
}

void Anemometer::mainThread(void *arg)
{
	_this = (Anemometer*) arg;
	HAL_TIM_RegisterCallback(&htim2, HAL_TIM_PERIOD_ELAPSED_CB_ID,
			overflowCallback);
	HAL_TIM_Base_Start_IT(&htim2);

	_this->MPS = 0.0;
	sample_time sample;
	for (;;)
	{
		if (osMessageQueueGet(_this->sampleQueue, &sample, 0, 10) == osOK)
		{
			_this->samples.push_back(
			{ sample.timestamp, sample.sample });
		}

		auto it = _this->samples.begin();
		int count = 0;
		double sum = 0.0;

		while (it != _this->samples.end())
		{
			if ((HAL_GetTick() - it->first) > 1000)
			{
				it = _this->samples.erase(it);
			}
			else
			{
				count++;
				sum += it->second;
				++it;
			}
		}

		if (count != 0)
		{
			_this->MPS = sum / count;
		}
		else
		{
			_this->MPS = 0.0;
		}

		if (_this->MPS < 0.5)
		{
			_this->MPS = 0.0;
		}

		osDelay(5);
	}
}

double Anemometer::getAverageSpeed()
{
	return MPS;
}

void Anemometer::adc_cb(ADC_HandleTypeDef *hadc)
{
	// Notify thread that conversion is complete
	osEventFlagsSet(conversionFlags, 0x01);
}

double Anemometer::getAngle()
{

	double retval = 0.0;
	if(osMutexAcquire(this->adcLock, 200) == osOK)
	{
		ADC_ChannelConfTypeDef sConfig =
		{ 0 };
		sConfig.Channel = ADC_CHANNEL_8;
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
			double wind = (first * 180.0) / 2048.0;

			if (wind > 180.0)
			{
				wind = wind - 360.0;
			}
			retval = wind;
		}
		osMutexRelease(this->adcLock);
	}
		return retval;
}

Anemometer::~Anemometer()
{
	// TODO Auto-generated destructor stub
}

