/*
 * CANOpenTask.cpp
 *
 *  Created on: Oct 1, 2021
 *      Author: reedt
 */

#include "CANopenNode/CANopen.h"
#include "CANopenNode/301/CO_ODinterface.h"
#include "CANopenNode/305/CO_LSSslave.h"
#include "bsp/DS28CM00ID/DS28CM00ID.h"
#include "bsp/LEDs/LEDManager.h"
#include "can.h"
#include "tim.h"
#include "CANOpenNode/OD.h"
#include "Logging/Logger.h"
#include "cmsis_os.h"
#include <map>
#include <set>
#include <functional>


#define NMT_CONTROL \
            CO_NMT_ERR_ON_ERR_REG \
          | CO_ERR_REG_GENERIC_ERR \
          | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 500
#define SDO_SRV_TIMEOUT_TIME 1000
#define SDO_CLI_TIMEOUT_TIME 500
#define SDO_CLI_BLOCK false
#define OD_STATUS_BITS NULL

#define COMMSPC 1
#define SENSORSPC 2
#define BMEPC 3



CO_t *canOpenHandle;
extern LEDManager leds;
extern DS28CM00_ID id1;

static uint8_t pendingID = CO_LSS_NODE_ID_ASSIGNMENT;
static uint16_t pendingRate = 1000;


void canopen_start(void)
{
	CO_ReturnError_t err;
	auto co = CO_new( /*config*/nullptr, /*heap used*/nullptr);
	err = CO_CANinit(co, (void*) &hcan1, 1000);
	uint64_t serNum = id1.getID();

	auto entry = OD_ENTRY_H1018_identity;
	OD_set_u32(entry, 1, 0xFFFFFFFF, true);	// Set device vendor id
	OD_set_u32(entry, 2, 0x00000003, true);	// Set product code (BME)
	OD_set_u32(entry, 3, 0x00000001, true);	// Set revision number (Comms V1)
	OD_set_u32(entry, 4, serNum, true);		// Set device serial number

	//	@formatter:off
	CO_LSS_address_t lssAddress =
	{ .identity =
	{ 		.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID,
			.productCode = OD_PERSIST_COMM.x1018_identity.productCode,
			.revisionNumber = OD_PERSIST_COMM.x1018_identity.revisionNumber,
			.serialNumber = OD_PERSIST_COMM.x1018_identity.serialNumber
	} };
	//	@formatter:on
	err = CO_LSSinit(co, &lssAddress, &pendingID, &pendingRate);

	uint32_t errInf = 0;
	//	@formatter:off
	err = CO_CANopenInit(co,
			nullptr, /* alternate NMT */
			nullptr, /* alternate em */
			OD, /* Object dictionary */
			OD_STATUS_BITS, /* Optional OD_statusBits */
			(CO_NMT_control_t)(NMT_CONTROL),
			FIRST_HB_TIME, /* ??? firstHBTime_ms */
			SDO_SRV_TIMEOUT_TIME, /*??? SDOserverTimeoutTime_ms */
			SDO_CLI_TIMEOUT_TIME, /*??? SDOclientTimeoutTime_ms */
			SDO_CLI_BLOCK, /*??? SDOclientBlockTransfer */
			pendingID, /*??? CO_activeNodeId */
			&errInf /*errInfo*/);
	//	@formatter:on

//	err = CO_CANopenInitPDO(co, co->em, OD, ID, &errInf);	// Delay until after Node-ID assignment

	CO_CANsetNormalMode(co->CANmodule);

	canOpenHandle = co;

	HAL_TIM_Base_Start_IT(&htim15);
	HAL_TIM_Base_Start_IT(&htim16);
	__HAL_TIM_SET_COUNTER(&htim16, 0);
}


void canopen_oneMs(void)
{
	uint32_t us = __HAL_TIM_GET_COUNTER(&htim16);
	auto reset = CO_RESET_NOT;

	uint32_t next = 0;
	reset = CO_process(canOpenHandle, false, us, &next);

	switch (reset)
	{
	/** 0, Normal return, no action */
	case CO_RESET_NOT:
		break;
		/** 1, Application must provide communication reset. */
	case CO_RESET_COMM:
		HAL_TIM_Base_Stop_IT(&htim15);
		CO_CANsetConfigurationMode((void*) &hcan1);
		CO_delete(canOpenHandle);
		canopen_start();
		return;
		/** 2, Application must provide complete device reset */
	case CO_RESET_APP:
		HAL_NVIC_SystemReset(); // DOES NOT RETURN
		break;
		/** 3, Application must quit, no reset of microcontroller (command is not
		 * requested by the stack.) */
	case CO_RESET_QUIT:
		CO_CANsetConfigurationMode((void*) &hcan1);
		CO_delete(canOpenHandle);
		HAL_TIM_Base_Stop_IT(&htim15);
		return;
	default:
		Error_Handler();
	}

	bool syncWas = false;
#if (CO_CONFIG_SYNC) & CO_CONFIG_SYNC_ENABLE
            syncWas = CO_process_SYNC(canOpenHandle, us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_RPDO_ENABLE
            CO_process_RPDO(canOpenHandle, syncWas, us, NULL);
#endif
#if (CO_CONFIG_PDO) & CO_CONFIG_TPDO_ENABLE
            CO_process_TPDO(canOpenHandle, syncWas, us, NULL);
#endif

	auto LED_red = CO_LED_RED(canOpenHandle->LEDs, CO_LED_CANopen);
	auto LED_green = CO_LED_GREEN(canOpenHandle->LEDs, CO_LED_CANopen);
	if (LED_red != 0)
	{
		leds.turnOn(color::RED);
	}
	else
	{
		leds.turnOff(color::RED);
	}
	if (LED_green != 0)
	{
		leds.turnOn(color::GREEN);
	}
	else
	{
		leds.turnOff(color::GREEN);
	}
	__HAL_TIM_SET_COUNTER(&htim16, 0);
}

