/*
 * UARTManager.cpp
 *
 *  Created on: Jul 29, 2021
 *      Author: reedt
 */

#include "Logging/UARTManager.h"

#include <cstdio>
#include <string.h>


typedef struct
{
	char * str;
	size_t length;
} message;


UARTManager::UARTManager(UART_HandleTypeDef * uart)
{
	this->uart = uart;
}


__NO_RETURN void UARTManager::main(void* arg)
{
	UARTManager * manager = (UARTManager *) arg;
	char * msg;
	for(;;)
	{
		auto retval = osMessageQueueGet(manager->UARTOutboxHandle, &msg, nullptr,
		osWaitForever);
		if (retval == osOK)
		{
			int size = strlen(msg);
			HAL_UART_Transmit(manager->uart, (uint8_t*) msg, size, -1); // TODO: There is a macro for no timeout in HAL, need to find it
			vPortFree(msg);
		}
		osDelay(1);
	}
	osThreadExit();
}


void UARTManager::start()
{

	ProcessUART_attributes.priority = (osPriority_t) osPriorityNormal;
	ProcessUART_attributes.stack_size = 2048 * 4;

	/* creation of UARTOutbox */
	UARTOutboxHandle = osMessageQueueNew(16, sizeof(char *),
			&UARTOutbox_attributes);

	/* creation of ProcessUART */
	ProcessUARTHandle = osThreadNew(this->main, (void*) this,
			&ProcessUART_attributes);
}

void UARTManager::print(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	char *buffer = (char *) pvPortMalloc(vsnprintf(nullptr, 0, fmt, args) + 1);
	va_end (args);
	va_start(args, fmt);
	vsprintf(buffer, fmt, args);
	va_end (args);

	osMessageQueuePut(UARTOutboxHandle, &buffer, 0, 0);
}

void UARTManager::vprint(const char *format, va_list arg)
{
	char *buffer = (char *) pvPortMalloc(vsnprintf(nullptr, 0, format, arg) + 1);
	vsprintf(buffer, format, arg);
	osMessageQueuePut(UARTOutboxHandle, &buffer, 0, 0);
}

UARTManager::~UARTManager()
{

}

