/*
 * canbus.h
 *
 *  Created on: May 17, 2019
 *      Author: sceaj
 */

#ifndef CANBUS_H_
#define CANBUS_H_

#include "fsl_flexcan.h"

#define CAN_STD_ID(id) (id >> CAN_ID_STD_SHIFT)

typedef struct _can_data {
	flexcan_frame_t canFrame;
	long timestamp;
} can_data_t;

void CAN_Enable();

void CAN_Disable();

status_t CAN_GetLastStatus();

status_t CAN_StartReceiving();

void CAN_SendFrame(flexcan_frame_t* frame);

int CAN_IsFrameAvailable();

can_data_t* CAN_NextFrame();

void CAN_PrintStatus();

#endif /* CANBUS_H_ */
