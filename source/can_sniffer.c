/*
 * Copyright 2016-2018 NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    can_sniffer.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "fsl_common.h"
#include "fsl_flexcan.h"
#include "fsl_gpio.h"
#include "fsl_sd.h"
#include "fsl_sd_disk.h"
#include "sdlogger.h"

/* TODO: insert other definitions and declarations here. */
static const sdmmchost_detect_card_t s_sdCardDetect = {
    .cdType = kSDMMCHOST_DetectCardByHostDATA3,
    .cdTimeOut_ms = (86400000U),
};

static char logBuffer[128];
static char* logBufferNext;

// Number of frames we can buffer
#define FLEXCAN_FIFO_BUFFER_SIZE 64
#define FLEXCAN_FIFO_AVL (fifoReadIndex != fifoWriteIndex)
#define FLEXCAN_FIFO_BUFFER_ADVANCE(idx) ((idx + 1) % FLEXCAN_FIFO_BUFFER_SIZE)
#define FLEXCAN_FIFO_BUFFER_FULL (fifoReadIndex == FLEXCAN_FIFO_BUFFER_ADVANCE(fifoWriteIndex))


static uint32_t rxFilters[] = {
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x242, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x245, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x246, 0, 0),
    FLEXCAN_RX_FIFO_STD_FILTER_TYPE_A(0x441, 0, 0)
};
void* g_flexcanRxFilters;
volatile bool rxFifoFrameBufferOverflow = false;
volatile bool rxFifoOverflow = false;
volatile bool rxFifoWarning = false;
static flexcan_frame_t rxFifoBuffer[FLEXCAN_FIFO_BUFFER_SIZE];
volatile int fifoReadIndex = 0;
volatile int fifoWriteIndex = 0;
static flexcan_frame_t txFrame;
static flexcan_fifo_transfer_t fifoTransfer;
static uint32_t dataCount;
static uint32_t remoteCount;
volatile long timerTicks = 0L;

#define FRAMEID_TO_STD_ID(x) ((uint16_t)(x / 0x40000U))

void Timer_IRQ(void)
{
	++timerTicks;
}

static void delay(uint32_t delay_ms)
{
	uint64_t delay_count = MSEC_TO_COUNT(delay_ms, BOARD_BOOTCLOCKRUN_CORE_CLOCK) >> 4;
	while (delay_count--) __NOP();
}

status_t initSDcard(void)
{
    /* Set SD configuration information. */
    g_sd.host.base = SDHC;
    g_sd.host.capability.flags = kSDHC_SupportV330Flag | kSDHC_Support4BitFlag;
    g_sd.host.config.cardDetectDat3 = true;
    g_sd.host.sourceClock_Hz = BOARD_SDHC_CLK_FREQ;
    g_sd.busClock_Hz = BOARD_SDHC_CLK_FREQ;
    g_sd.flags = kSD_Support4BitWidthFlag;
    g_sd.operationVoltage = kCARD_OperationVoltage330V;
    /* card detect type */
    g_sd.usrParam.cd = &s_sdCardDetect;
#if defined DEMO_SDCARD_POWER_CTRL_FUNCTION_EXIST
    g_sd.usrParam.pwr = &s_sdCardPwrCtrl;
#endif
    return SD_Init(&g_sd);
}

void logCanData(flexcan_frame_t* canFrame) {
	if (canFrame->type == kFLEXCAN_FrameTypeData) {
		uint16_t stdId = FRAMEID_TO_STD_ID(canFrame->id);
        appendLog("$CAN,%ld.%02ld,%03X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
        		timerTicks / 100L,
				timerTicks % 100L,
				stdId,
				canFrame->dataByte0,
				canFrame->dataByte1,
				canFrame->dataByte2,
				canFrame->dataByte3,
				canFrame->dataByte4,
				canFrame->dataByte5,
				canFrame->dataByte6,
				canFrame->dataByte7);
		dataCount++;
	} else {
		remoteCount++;
	}
}

void sendCanTestFrame() {
	txFrame.format = kFLEXCAN_FrameFormatStandard;
	txFrame.type   = kFLEXCAN_FrameTypeData;
	txFrame.id     = FLEXCAN_ID_STD(0x7DF);
	txFrame.length = 8;
	txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x02) |
	                    CAN_WORD0_DATA_BYTE_1(0x01) |
	                    CAN_WORD0_DATA_BYTE_2(0x0C) |
	                    CAN_WORD0_DATA_BYTE_3(0x00);
	txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) |
	                    CAN_WORD1_DATA_BYTE_5(0xCC) |
	                    CAN_WORD1_DATA_BYTE_6(0x55) |
	                    CAN_WORD1_DATA_BYTE_7(0xCC);
	FLEXCAN_TransferSendBlocking(FLEXCAN_1_PERIPHERAL, 8, &txFrame);
}


void flexcanCallback(CAN_Type *base, flexcan_handle_t *handle, status_t status, uint32_t reason, void *userData)
{
	if (kStatus_FLEXCAN_RxFifoIdle == status) {
		if (FLEXCAN_FIFO_BUFFER_FULL) {
			rxFifoFrameBufferOverflow = true;
		} else {
			fifoWriteIndex = FLEXCAN_FIFO_BUFFER_ADVANCE(fifoWriteIndex);
			fifoTransfer.frame = &rxFifoBuffer[fifoWriteIndex];
		}
		FLEXCAN_TransferReceiveFifoNonBlocking(FLEXCAN_1_PERIPHERAL, &FlexCAN_1_handle, &fifoTransfer);
	}
	if (kStatus_FLEXCAN_RxFifoOverflow == status) rxFifoOverflow = true;
	if (kStatus_FLEXCAN_RxFifoWarning == status) rxFifoWarning = true;
}

/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    g_flexcanRxFilters = rxFilters;
    FlexCAN_1_rx_fifo_config.idFilterNum = 4;
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

	delay(5000U);
    PRINTF("SD Card / FatFS Demo\r\n");
    while (initSDcard() != kStatus_Success)
    {
    	PRINTF("Could not initialize SD card. Trying again...\r\n");
    	delay(500U);
    }

    delay(100U);
    PRINTF("Mounting SD filesystem...\r\n");
    while (mountSDfilesystem() != kStatus_Success)
    {
    	PRINTF("Could not mount SD filesystem.  Trying again...\r\n");
    	delay(500U);
    }

    delay(100U);
    openLogger("CAN");

    FLEXCAN_Enable(FLEXCAN_1_PERIPHERAL, true);
    PRINTF("FLEXCAN module enabled.\r\n");

    /* Force the counter to be placed into memory. */
    volatile static int i = 0;
    int j = 0;
    logBufferNext = logBuffer;
    /* Enter an infinite loop, just incrementing a counter. */
    fifoTransfer.frame = &rxFifoBuffer[fifoWriteIndex];
	FLEXCAN_TransferReceiveFifoNonBlocking(FLEXCAN_1_PERIPHERAL, &FlexCAN_1_handle, &fifoTransfer);
    while (1) {

    	while (FLEXCAN_FIFO_AVL) {
    		logCanData(&rxFifoBuffer[fifoReadIndex]);
    		fifoReadIndex = FLEXCAN_FIFO_BUFFER_ADVANCE(fifoReadIndex);
    	}

        i++ ;
        if ((i % 0x100000) == 0) {
        	if (rxFifoFrameBufferOverflow) {
        		PRINTF("CAN fifo frame buffer is FULL!\r\n");
        		rxFifoFrameBufferOverflow = false;
        	} else if (rxFifoWarning) {
        		PRINTF("Fifo WARNING detected!\r\n");
        		rxFifoWarning = false;
        	} else if (rxFifoOverflow) {
        		PRINTF("Fifo OVERFLOW detected!\r\n");
        		rxFifoOverflow = false;
        	}
            PRINTF("Iter: %d; Frames: %d\r\n", i, dataCount);
        	// Flash the led...
        	if ((i / 0x100000) & 0x01) {
            	GPIO_PinWrite(GPIOC, 5U, 1);
        	} else {
            	GPIO_PinWrite(GPIOC, 5U, 0);
        	}
        	j++;
        }

//        if ((j % 10) == 0) {
//        	PRINTF("Sending CAN Frame...\r\n");
//        	sendCanTestFrame();
//        	j++;
//        }
    }

    FLEXCAN_Enable(FLEXCAN_1_PERIPHERAL, false);
    closeLogger();
    return 0 ;
}
