/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "adc.h"
#include "can.h"
#include "crc.h"
#include "dma.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <cstdint>
#include <stdio.h>

#include "CANopenNode/301/CO_ODinterface.h"
#include "Logging/UARTLogHandler.h"

#include "device_specific/BME280/BME280.h"
#include "device_specific/Anemometer/Anemometer.h"
#include "device_specific/RainDetector/RainDetector.h"

#include "bsp/DS28CM00ID/DS28CM00ID.h"
#include "bsp/LEDs/LEDManager.h"
#include "bsp/UART/UARTManager.h"

#include "CANOpenNode/OD.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern DS28CM00_ID id1;
extern LEDManager leds;
extern UARTManager uartMan;
extern Logger Log;

/* Definitions for SerialID_lock */
osMutexId_t SerialID_lockHandle;
const osMutexAttr_t SerialID_lock_attributes =
{ .name = "SerialID_lock" };

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern void canopen_start(void);
void measurementTask(void*);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	 configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
	 function that will get called if a call to pvPortMalloc() fails.
	 pvPortMalloc() is called internally by the kernel whenever a task, queue,
	 timer or semaphore is created. It is also called by various parts of the
	 demo application. If heap_1.c or heap_2.c are used, then the size of the
	 heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	 FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	 to query the size of free heap space that remains (although it does not
	 provide information on how the remaining heap might be fragmented). */
}

/* USER CODE END 5 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */
	/* Create the mutex(es) */

	/* creation of SerialID_lock */
	SerialID_lockHandle = osMutexNew(&SerialID_lock_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */

	/* USER CODE BEGIN RTOS_TIMERS */

	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	osThreadNew(measurementTask, nullptr, nullptr);
	/* USER CODE END RTOS_THREADS */

	/* creation of MeasureEvent */

	/* creation of ButtonEvent */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	uartMan.start();
	leds.start(false);
	Log.info("Application Started");
	canopen_start();
	/* USER CODE END RTOS_EVENTS */

}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void measurementTask(void*)
{
	Logger ilog("MeasureTask");

	ilog.info("Starting measurements");

	const osMutexAttr_t ADC_lock_attributes =
	{ .name = "ADC_lock" };
	osMutexId_t ADC_lockHandle = osMutexNew(&ADC_lock_attributes);

	Anemometer wind(ADC_lockHandle);
	RainDetector rain(ADC_lockHandle);

	BME280 bme1(&hspi1, BME_CS_GPIO_Port, BME_CS_Pin);

	bool initalized = bme1.init();
	if (!initalized)
	{
		ilog.error("BME initialization failed");
		for (;;)
			osDelay(1000);
	}

	float pressure = 0.0;
	auto pressEntry = OD_ENTRY_H6001_pressure;

	float humidity = 0.0;
	auto humEntry = OD_ENTRY_H6000_humidity;

	float windSpeed = 0.0;
	auto winSpdEntry = OD_ENTRY_H6002_windSpeed;

	float windDirection = 0.0;
	auto winDirEntry = OD_ENTRY_H6003_windDirection;

	bool rainDetected = false;
	auto rainDetEntry = OD_ENTRY_H6004_rainDetection;

	for (;;)
	{
		pressure = bme1.readPressure();
		humidity = bme1.readHumidity();

		windSpeed = wind.getAverageSpeed();
		windDirection = wind.getAngle();

		rainDetected = rain.isRaining();

		ilog.info(
				"Pressure: %8.8f pA, Humidity: %8.8f%%, Wind Speed: %8.8f m/s, Wind Direction: %8.8f* from north, Rain: %s",
				pressure, humidity, windSpeed, windDirection,
				rainDetected ? "Yes" : "No");

		OD_set_f32(pressEntry, 0, pressure, true);

		OD_set_f32(humEntry, 0, humidity, true);

		OD_set_f32(winSpdEntry, 0, windSpeed, true);

		OD_set_f32(winDirEntry, 0, windDirection, true);

		OD_set_i8(rainDetEntry, 0, rainDetected, true);

		osDelay(500);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
