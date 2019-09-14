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

void CAN_Enable();

void CAN_Disable();

status_t CAN_GetLastStatus();

status_t CAN_StartReceiving();

void CAN_SendFrame(flexcan_frame_t* frame);

int CAN_IsFrameAvailable();

flexcan_frame_t* CAN_NextFrame();

void CAN_PrintStatus();

#endif /* CANBUS_H_ */
