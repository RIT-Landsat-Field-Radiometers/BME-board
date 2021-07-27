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
#include <cstdint>
#include <stdio.h>
#include "BME280.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	char *string;
	int length;
} out_mesg;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Definitions for processCANMesg */
osThreadId_t processCANMesgHandle;
osThreadAttr_t processCANMesg_attributes;
/* Definitions for MeasureTask */
osThreadId_t MeasureTaskHandle;
osThreadAttr_t MeasureTask_attributes;
/* Definitions for sysMonitor */
osThreadId_t sysMonitorHandle;
osThreadAttr_t sysMonitor_attributes;
/* Definitions for UI */
osThreadId_t UIHandle;
osThreadAttr_t UI_attributes;
/* Definitions for processUART */
osThreadId_t processUARTHandle;
osThreadAttr_t processUART_attributes;
/* Definitions for CANInbox */
osMessageQueueId_t CANInboxHandle;
osMessageQueueAttr_t CANInbox_attributes;
/* Definitions for CANOutbox */
osMessageQueueId_t CANOutboxHandle;
osMessageQueueAttr_t CANOutbox_attributes;
/* Definitions for UARTOutbox */
osMessageQueueId_t UARTOutboxHandle;
osMessageQueueAttr_t UARTOutbox_attributes;
/* Definitions for MeasureTimer */
osTimerId_t MeasureTimerHandle;
osTimerAttr_t MeasureTimer_attributes;
/* Definitions for LEDTimeoutTimer */
osTimerId_t LEDTimeoutTimerHandle;
osTimerAttr_t LEDTimeoutTimer_attributes;
/* Definitions for InputTimeout */
osTimerId_t InputTimeoutHandle;
osTimerAttr_t InputTimeout_attributes;
/* Definitions for BME280_lock */
osMutexId_t BME280_lockHandle;
osMutexAttr_t BME280_lock_attributes;
/* Definitions for SiliconID_lock */
osMutexId_t SiliconID_lockHandle;
osMutexAttr_t SiliconID_lock_attributes;
/* Definitions for startMeasurement */
osSemaphoreId_t startMeasurementHandle;
osSemaphoreAttr_t startMeasurement_attributes;
/* Definitions for LEDState */
osEventFlagsId_t LEDStateHandle;
osEventFlagsAttr_t LEDState_attributes;
/* Definitions for ButtonCommand */
osEventFlagsId_t ButtonCommandHandle;
osEventFlagsAttr_t ButtonCommand_attributes;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartCANMesgTask(void *argument);
void StartMeasureTask(void *argument);
void startMonitor(void *argument);
void startUI(void *argument);
void startUART(void *argument);
void MeasureRoutine(void *argument);
void LEDTimeoutRoutine(void *argument);
void processInput(void *argument);

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
__weak void configureTimerForRunTimeStats(void) {

}

__weak unsigned long getRunTimeCounterValue(void) {
	return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void) {
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
void vApplicationTickHook(void) {
	/* This function will be called by each tick interrupt if
	 configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
	 added here, but the tick hook is called from an interrupt context, so
	 code must not attempt to block, and only the interrupt safe FreeRTOS API
	 functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName) {
	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
	 called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void) {
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
void MX_FREERTOS_Init(void) {
	/* USER CODE BEGIN Init */

	processCANMesg_attributes.name = "processCANMesg";
	processCANMesg_attributes.stack_size = 2048 * 4;
	processCANMesg_attributes.priority = (osPriority_t) osPriorityNormal;

	MeasureTask_attributes.name = "MeasureTask";
	MeasureTask_attributes.stack_size = 2048 * 4;
	MeasureTask_attributes.priority = (osPriority_t) osPriorityNormal;

	sysMonitor_attributes.name = "sysMonitor";
	sysMonitor_attributes.stack_size = 2048 * 4;
	sysMonitor_attributes.priority = (osPriority_t) osPriorityLow;

	UI_attributes.name = "UI";
	UI_attributes.stack_size = 2048 * 4;
	UI_attributes.priority = (osPriority_t) osPriorityLow;

	processUART_attributes.name = "processUART";
	processUART_attributes.stack_size = 2048 * 4;
	processUART_attributes.priority = (osPriority_t) osPriorityNormal;

	CANInbox_attributes.name = "CANInbox";
	CANOutbox_attributes.name = "CANOutbox";
	MeasureTimer_attributes.name = "MeasureTimer";
	LEDTimeoutTimer_attributes.name = "LEDTimeoutTimer";
	InputTimeout_attributes.name = "InputTimeout";
	BME280_lock_attributes.name = "BME280_lock";
	SiliconID_lock_attributes.name = "SiliconID_lock";
	startMeasurement_attributes.name = "startMeasurement";
	LEDState_attributes.name = "LEDState";
	ButtonCommand_attributes.name = "ButtonCommand";

	/* USER CODE END Init */
	/* Create the mutex(es) */
	/* creation of BME280_lock */
	BME280_lockHandle = osMutexNew(&BME280_lock_attributes);

	/* creation of SiliconID_lock */
	SiliconID_lockHandle = osMutexNew(&SiliconID_lock_attributes);

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Create the semaphores(s) */
	/* creation of startMeasurement */
	startMeasurementHandle = osSemaphoreNew(1, 1, &startMeasurement_attributes);

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* Create the timer(s) */
	/* creation of MeasureTimer */
	MeasureTimerHandle = osTimerNew(MeasureRoutine, osTimerPeriodic, NULL,
			&MeasureTimer_attributes);

	/* creation of LEDTimeoutTimer */
	LEDTimeoutTimerHandle = osTimerNew(LEDTimeoutRoutine, osTimerOnce, NULL,
			&LEDTimeoutTimer_attributes);

	/* creation of InputTimeout */
	InputTimeoutHandle = osTimerNew(processInput, osTimerOnce, NULL,
			&InputTimeout_attributes);

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of CANInbox */
	CANInboxHandle = osMessageQueueNew(16, sizeof(uint32_t),
			&CANInbox_attributes);

	/* creation of CANOutbox */
	CANOutboxHandle = osMessageQueueNew(16, sizeof(uint32_t),
			&CANOutbox_attributes);

	/* creation of UARTOutbox */
	UARTOutboxHandle = osMessageQueueNew(16, sizeof(uint32_t),
			&UARTOutbox_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of processCANMesg */
	processCANMesgHandle = osThreadNew(StartCANMesgTask, nullptr,
			&processCANMesg_attributes);

	/* creation of MeasureTask */
	MeasureTaskHandle = osThreadNew(StartMeasureTask, nullptr,
			&MeasureTask_attributes);

	/* creation of sysMonitor */
	sysMonitorHandle = osThreadNew(startMonitor, nullptr,
			&sysMonitor_attributes);

	/* creation of UI */
	UIHandle = osThreadNew(startUI, nullptr, &UI_attributes);

	/* creation of processUART */
	processUARTHandle = osThreadNew(startUART, nullptr,
			&processUART_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* creation of LEDState */
	LEDStateHandle = osEventFlagsNew(&LEDState_attributes);

	/* creation of ButtonCommand */
	ButtonCommandHandle = osEventFlagsNew(&ButtonCommand_attributes);

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartCANMesgTask */
/**
 * @brief  Function implementing the processCANMesg thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCANMesgTask */
void StartCANMesgTask(void *argument) {
	/* USER CODE BEGIN StartCANMesgTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartCANMesgTask */
}

/* USER CODE BEGIN Header_StartMeasureTask */
/**
 * @brief Function implementing the MeasureTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartMeasureTask */
void StartMeasureTask(void *argument) {
	/* USER CODE BEGIN StartMeasureTask */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END StartMeasureTask */
}

/* USER CODE BEGIN Header_startMonitor */
/**
 * @brief Function implementing the sysMonitor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startMonitor */
void startMonitor(void *argument) {
	/* USER CODE BEGIN startMonitor */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END startMonitor */
}

/* USER CODE BEGIN Header_startUI */
/**
 * @brief Function implementing the UI thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startUI */
void startUI(void *argument) {
	/* USER CODE BEGIN startUI */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END startUI */
}

/* USER CODE BEGIN Header_startUART */
/**
 * @brief Function implementing the processUART thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_startUART */
void startUART(void *argument) {
	/* USER CODE BEGIN startUART */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END startUART */
}

/* MeasureRoutine function */
void MeasureRoutine(void *argument) {
	/* USER CODE BEGIN MeasureRoutine */

	/* USER CODE END MeasureRoutine */
}

/* LEDTimeoutRoutine function */
void LEDTimeoutRoutine(void *argument) {
	/* USER CODE BEGIN LEDTimeoutRoutine */

	/* USER CODE END LEDTimeoutRoutine */
}

/* processInput function */
void processInput(void *argument) {
	/* USER CODE BEGIN processInput */

	/* USER CODE END processInput */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
