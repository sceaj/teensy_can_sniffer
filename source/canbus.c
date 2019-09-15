/*
 * canbus.c
 *
 *  Created on: May 17, 2019
 *      Author: Jeffrey
 */
#include "canbus.h"
#include "fifobuffer.h"
#include "fsl_debug_console.h"
#include "peripherals.h"


/* Flexcan data structures */
static uint32_t rxFilters[] = {
	FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x0C0, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x140, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x240, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x300, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x440, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x5F0, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x6F0, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x7F0, 0, 0)
};
void* g_rxFifoFilters = rxFilters;

volatile bool canRxFifoBufferOverflow = false;
volatile bool flexcanRxFifoOverflow = false;
volatile bool flexcanRxFifoWarning = false;

#define CAN_FIFO_BUFFER_SIZE 512
static flexcan_frame_t canRxFifoBuffer[CAN_FIFO_BUFFER_SIZE];
volatile int canFifoBufferRead = 0;
volatile int canFifoBufferWrite = 0;

static flexcan_fifo_transfer_t flexcanFifoTransfer;
volatile int canTxCount;
volatile int canCallbackCount;


/*
 * FLEXCAN module support methods
 *
 * Callback method
 */
void flexcanCallback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t result, void *userData)
{
	if (kStatus_FLEXCAN_RxFifoIdle == status) {
		if (FIFO_BUFFER_FULL(canFifoBufferRead,canFifoBufferWrite,CAN_FIFO_BUFFER_SIZE)) {
			canRxFifoBufferOverflow = true;
		} else {
			canFifoBufferWrite = FIFO_BUFFER_ADVANCE(canFifoBufferWrite, CAN_FIFO_BUFFER_SIZE);
			flexcanFifoTransfer.frame = &canRxFifoBuffer[canFifoBufferWrite];
		}
		FLEXCAN_TransferReceiveFifoNonBlocking(FLEXCAN_1_PERIPHERAL, &FlexCAN_1_handle, &flexcanFifoTransfer);
	}
	if ((kStatus_FLEXCAN_TxIdle == status)
			&& (8 == result)) {
		canTxCount++;
	}
	if (kStatus_FLEXCAN_RxFifoOverflow == status) flexcanRxFifoOverflow = true;
	if (kStatus_FLEXCAN_RxFifoWarning == status) flexcanRxFifoWarning = true;
	canCallbackCount++;
}
/* END FLEXCAN module callback method */


void CAN_Enable()
{
    FLEXCAN_Enable(FLEXCAN_1_PERIPHERAL, true);
}

void CAN_Disable()
{
    FLEXCAN_Enable(FLEXCAN_1_PERIPHERAL, false);
}

status_t CAN_StartReceiving()
{
    flexcanFifoTransfer.frame = &canRxFifoBuffer[canFifoBufferWrite];
	return FLEXCAN_TransferReceiveFifoNonBlocking(FLEXCAN_1_PERIPHERAL, &FlexCAN_1_handle, &flexcanFifoTransfer);
}

void CAN_SendTxFrame()
{
	FLEXCAN_TransferSendNonBlocking(FLEXCAN_1_PERIPHERAL, &FlexCAN_1_handle, (flexcan_mb_transfer_t *)&FLEXCAN_1_tx_mb_transfer);
}

int CAN_IsFrameAvailable()
{
	return FIFO_AVL(canFifoBufferRead,canFifoBufferWrite);
}

flexcan_frame_t* CAN_NextFrame()
{
	flexcan_frame_t* nextFrame = &canRxFifoBuffer[canFifoBufferRead];
	canFifoBufferRead = FIFO_BUFFER_ADVANCE(canFifoBufferRead,CAN_FIFO_BUFFER_SIZE);
	return nextFrame;
}

void CAN_PrintStatus()
{
	/* Send important CAN events to DebugConsole */
	if (canRxFifoBufferOverflow) {
		PRINTF("CAN fifo frame buffer is FULL!\r\n");
		canRxFifoBufferOverflow = false;
	}
	if (flexcanRxFifoWarning) {
		PRINTF("Fifo WARNING detected!\r\n");
		flexcanRxFifoWarning = false;
	}
	if (flexcanRxFifoOverflow) {
		PRINTF("Fifo OVERFLOW detected!\r\n");
		flexcanRxFifoOverflow = false;
	}
}
