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
#include "BME280.h"
#include <LEDManager.h>

#include "Logging/Logger.h"
#include "Logging/UARTLogHandler.h"
#include "Logging/UARTManager.h"

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
extern BME280 bme1;
LEDManager leds(
{ .port = GPIOD, .pin = LED_R_Pin },
{ .port = GPIOD, .pin = LED_G_Pin },
{ .port = GPIOD, .pin = LED_B_Pin });

UARTManager uartMan(&huart1);

/* Definitions for MeasureTask */
osThreadId_t MeasureTaskHandle;
osThreadAttr_t MeasureTask_attributes =
{ .name = "MeasureTask", };
/* Definitions for ProcessCAN */
osThreadId_t ProcessCANHandle;
osThreadAttr_t ProcessCAN_attributes =
{ .name = "ProcessCAN", };
/* Definitions for SysMonitor */
osThreadId_t SysMonitorHandle;
osThreadAttr_t SysMonitor_attributes =
{ .name = "SysMonitor", };

/* Definitions for CANInbox */
osMessageQueueId_t CANInboxHandle;
const osMessageQueueAttr_t CANInbox_attributes =
{ .name = "CANInbox" };
/* Definitions for CANOutbox */
osMessageQueueId_t CANOutboxHandle;
const osMessageQueueAttr_t CANOutbox_attributes =
{ .name = "CANOutbox" };

/* Definitions for MeasureTimer */
osTimerId_t MeasureTimerHandle;
const osTimerAttr_t MeasureTimer_attributes =
{ .name = "MeasureTimer" };

/* Definitions for ButtonTimeout */
osTimerId_t ButtonTimeoutHandle;
const osTimerAttr_t ButtonTimeout_attributes =
{ .name = "ButtonTimeout" };
/* Definitions for BME280_lock */
osMutexId_t BME280_lockHandle;
const osMutexAttr_t BME280_lock_attributes =
{ .name = "BME280_lock" };
/* Definitions for SerialID_lock */
osMutexId_t SerialID_lockHandle;
const osMutexAttr_t SerialID_lock_attributes =
{ .name = "SerialID_lock" };
/* Definitions for MeasureEvent */
osEventFlagsId_t MeasureEventHandle;
const osEventFlagsAttr_t MeasureEvent_attributes =
{ .name = "MeasureEvent" };

/* Definitions for ButtonEvent */
osEventFlagsId_t ButtonEventHandle;
const osEventFlagsAttr_t ButtonEvent_attributes =
{ .name = "ButtonEvent" };

UARTLogHandler * handler(UARTLogHandler::configure(&uartMan, LOG_LEVEL_ALL));
Logger Log("app");

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void startMeasuring(void *argument);
void startCANIO(void *argument);
void startMonitoring(void *argument);
void startUI(void *argument);
void startUARTIO(void *argument);
void startMeasurement(void *argument);
void stopLEDs(void *argument);
void LEDSlowToggle(void *argument);
void LEDFastToggle(void *argument);
void ProcessButtons(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
	return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void)
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	 to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
	 task. It is essential that code added to this hook function never attempts
	 to block in any way (for example, call xQueueReceive() with a block time
	 specified, or call vTaskDelay()). If the application makes use of the
	 vTaskDelete() API function (as this demo application does) then it is also
	 important that vApplicationIdleHook() is permitted to return to its calling
	 function, because it is the responsibility of the idle task to clean up
	 memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook(void)
{
	/* This function will be called by each tick interrupt if
	 configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
	 added here, but the tick hook is called from an interrupt context, so
	 code must not attempt to block, and only the interrupt safe FreeRTOS API
	 functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

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

	MeasureTask_attributes.stack_size = 2048 * 4;
	MeasureTask_attributes.priority = (osPriority_t) osPriorityNormal;

	ProcessCAN_attributes.priority = (osPriority_t) osPriorityNormal;
	ProcessCAN_attributes.stack_size = 2048 * 4;

	SysMonitor_attributes.priority = (osPriority_t) osPriorityLow;
	SysMonitor_attributes.stack_size = 2048 * 4;

	/* USER CODE END Init */
	/* Create the mutex(es) */
	/* creation of BME280_lock */
	BME280_lockHandle = osMutexNew(&BME280_lock_attributes);

	/* creation of SerialID_lock */
	SerialID_lockHandle = osMutexNew(&SerialID_lock_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* creation of MeasureTimer */
	MeasureTimerHandle = osTimerNew(startMeasurement, osTimerPeriodic,
			(void*) nullptr, &MeasureTimer_attributes);

	/* creation of ButtonTimeout */
	ButtonTimeoutHandle = osTimerNew(ProcessButtons, osTimerOnce,
			(void*) nullptr, &ButtonTimeout_attributes);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	osTimerStart(MeasureTimerHandle, 1000); // Take a measurement every second

	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of CANInbox */
	CANInboxHandle = osMessageQueueNew(16, sizeof(uint32_t),
			&CANInbox_attributes);

	/* creation of CANOutbox */
	CANOutboxHandle = osMessageQueueNew(16, sizeof(uint32_t),
			&CANOutbox_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of MeasureTask */
	MeasureTaskHandle = osThreadNew(startMeasuring, (void*) nullptr,
			&MeasureTask_attributes);

	/* creation of ProcessCAN */
	ProcessCANHandle = osThreadNew(startCANIO, (void*) nullptr,
			&ProcessCAN_attributes);

	/* creation of SysMonitor */
	SysMonitorHandle = osThreadNew(startMonitoring, (void*) nullptr,
			&SysMonitor_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* creation of MeasureEvent */
	MeasureEventHandle = osEventFlagsNew(&MeasureEvent_attributes);

	/* creation of ButtonEvent */
	ButtonEventHandle = osEventFlagsNew(&ButtonEvent_attributes);

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	uartMan.start();
	leds.start();
	Log.info("Application Started");
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_startMeasuring */
/**
 * @brief  Function implementing the MeasureTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_startMeasuring */
void startMeasuring(void *argument)
{
	/* USER CODE BEGIN startMeasuring */
	Logger privLog("Sensors");
	/* Infinite loop */
	for (;;)
	{
		auto retval = osEventFlagsWait(MeasureEventHandle, 0x01, osFlagsWaitAny,
		osWaitForever);
		retval = osMutexAcquire(BME280_lockHandle, 1000);
		if (retval == osOK)
		{
			double press = bme1.readPressure();
			double hum = bme1.readHumidity();
			double temp = bme1.readTemperature();

			privLog.debug(
					"Pressure: %8.4f, Humidity: %8.4f, Temperature: %8.4f",
					press, hum, temp);

			privLog.debug(
					"Pressure: %8.4f, Humidity: %8.4f, Temperature: %8.4f",
					press, hum, temp);
			osMutexRelease(BME280_lockHandle);
		}
		else
		{
			privLog.error("Unable to acquire mutex");
		}
		osDelay(1);
	}
	/* USER CODE END startMeasuring */
}

/* USER CODE BEGIN Header_startCANIO */
/**
 * @brief Function implementing the ProcessCAN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startCANIO */
void startCANIO(void *argument)
{
	/* USER CODE BEGIN startCANIO */
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END startCANIO */
}

/* USER CODE BEGIN Header_startMonitoring */
/**
 * @brief Function implementing the SysMonitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startMonitoring */
void startMonitoring(void *argument)
{
	/* USER CODE BEGIN startMonitoring */
	Logger privLog("System");

	/* Infinite loop */
	for (;;)
	{
		leds.fastFlash(RED);

		privLog.debug("LEDs all fast");

		osDelay(5000);
		leds.slowFlash(GREEN);
		privLog.debug("LEDs all slow");

		osDelay(15000);
		leds.turnOn(BLUE);
		privLog.debug("LEDs all on");

		osDelay(5000);
		leds.turnOff(WHITE);
		privLog.debug("LEDs all off");

		size_t freeHeap = xPortGetFreeHeapSize();
		if (freeHeap < 2000)
		{
			privLog.error("LOW HEAP, %u bytes remaining",
					freeHeap);
		}
		osDelay(5000);
	}
	/* USER CODE END startMonitoring */
}

/* startMeasurement function */
void startMeasurement(void *argument)
{
	/* USER CODE BEGIN startMeasurement */
	osEventFlagsSet(MeasureEventHandle, 0x01);
	/* USER CODE END startMeasurement */
}

/* stopLEDs function */
void stopLEDs(void *argument)
{
	/* USER CODE BEGIN stopLEDs */

	/* USER CODE END stopLEDs */
}

/* ProcessButtons function */
void ProcessButtons(void *argument)
{
	/* USER CODE BEGIN ProcessButtons */

	/* USER CODE END ProcessButtons */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
