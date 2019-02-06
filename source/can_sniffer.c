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
 * @file    virtualcom.c
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
static flexcan_frame_t rxFrame;
static flexcan_frame_t txFrame;
static uint32_t dataCount;
static uint32_t remoteCount;

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
        appendLog("$CAN,%d,%04x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x\n",
        		canFrame->length,
        		canFrame->id,
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
	txFrame.id     = FLEXCAN_ID_STD(0x455);
	txFrame.length = 8;
	txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x10) |
	                    CAN_WORD0_DATA_BYTE_1(0x21) |
	                    CAN_WORD0_DATA_BYTE_2(0x32) |
	                    CAN_WORD0_DATA_BYTE_3(0x43);
	txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x54) |
	                    CAN_WORD1_DATA_BYTE_5(0x65) |
	                    CAN_WORD1_DATA_BYTE_6(0x80) |
	                    CAN_WORD1_DATA_BYTE_7(0xF5);
	/* Writes a transmit message buffer to send a CAN Message. */
	FLEXCAN_WriteTxMb(FLEXCAN_1_PERIPHERAL, 8, &txFrame);
	/* Waits until the transmit message buffer is empty. */
	while (!FLEXCAN_GetMbStatusFlags(FLEXCAN_1_PERIPHERAL, 1 << 8));
	/* Cleans the transmit message buffer empty status. */
	FLEXCAN_ClearMbStatusFlags(FLEXCAN_1_PERIPHERAL, 1 << 8);
}

/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
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
    logBufferNext = logBuffer;
    /* Enter an infinite loop, just incrementing a counter. */
    while (i < 1000000) {
    	if (FLEXCAN_GetMbStatusFlags(FLEXCAN_1_PERIPHERAL, kFLEXCAN_RxFifoFrameAvlFlag)) {
    		do {
        		/* Reads the message from the receive FIFO. */
        		FLEXCAN_ReadRxFifo(FLEXCAN_1_PERIPHERAL, &rxFrame);
        		logCanData(&rxFrame);
        		/* Cleans the receive FIFO available status. */
        		FLEXCAN_ClearMbStatusFlags(FLEXCAN_1_PERIPHERAL, kFLEXCAN_RxFifoFrameAvlFlag);
    		} while (FLEXCAN_GetMbStatusFlags(FLEXCAN_1_PERIPHERAL, kFLEXCAN_RxFifoFrameAvlFlag));
    	} else if (FLEXCAN_GetMbStatusFlags(FLEXCAN_1_PERIPHERAL, 1 << 9)) {
    		/* Reads the received message from the receive message buffer. */
    		FLEXCAN_ReadRxMb(FLEXCAN_1_PERIPHERAL, 9, &rxFrame);
    		logCanData(&rxFrame);
    		/* Cleans the receive message buffer full status. */
    		FLEXCAN_ClearMbStatusFlags(FLEXCAN_1_PERIPHERAL, 1 << 9);
    	} else {
    		delay(5U);
    	}

        i++ ;
        if ((i % 1000) == 0) {
            PRINTF("Iter: %d; Frames: %d\r\n", i, dataCount);
        	// Flash the led...
        	if ((i / 1000) & 0x01) {
            	GPIO_PinWrite(GPIOC, 5U, 1);
        	} else {
            	GPIO_PinWrite(GPIOC, 5U, 0);
        	}
        	PRINTF("Sending CAN Frame...\r\n");
        	sendCanTestFrame();
        }
    }

    FLEXCAN_Enable(FLEXCAN_1_PERIPHERAL, false);
    closeLogger();
    return 0 ;
}
