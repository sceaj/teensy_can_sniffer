/*
 * Copyright 2016-2018 NXP
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
 * @file    teensy_can_logger.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "accelerometer.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
/* TODO: insert other include files here. */
#include "canbus.h"
#include "fsl_gpio.h"
#include "fsl_sd.h"
#include "fsl_sd_disk.h"
#include "gps.h"
#include "sdlogger.h"

/* TODO: insert other definitions and declarations here. */
/* SD Card / FatFS / Logging data structures */
static const sdmmchost_detect_card_t s_sdCardDetect = {
    .cdType = kSDMMCHOST_DetectCardByHostDATA3,
    .cdTimeOut_ms = (86400000U),
};

static int accelDataCount;
static int canDataCount;
static int gpsDataCount;

/*
 * 100Hz Periodic Interrupt Timer
 */
volatile long timerTicks = 0L;

void PIT0_IRQHandler(void)
{
	++timerTicks;

	if (!(timerTicks % 5L)) {
		/* Start a new Accelerometer reading, ~20Hz */
		ACCEL_TriggerCollection();
	}

	PIT_ClearStatusFlags(PIT_1_PERIPHERAL, kPIT_Chnl_0, kPIT_TimerFlag);
}
/* END 100Hz PIT */

/*
 * Simple busy-wait delay function
 */
void _delay(uint32_t delay_ms)
{
	uint64_t delay_count = MSEC_TO_COUNT(delay_ms, BOARD_BOOTCLOCKRUN_CORE_CLOCK) >> 4;
	while (delay_count--) __NOP();
}
/* End delay function */

/*
 * SD Card initialization using the SDHC module
 */
static status_t initSDcard(void)
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


/* CAN Frame Logging */
void logCanData(can_data_t* canData) {
	if (canData->canFrame.type == kFLEXCAN_FrameTypeData) {
		uint16_t stdId = CAN_STD_ID(canData->canFrame.id);
        appendLog("$CNDRV,%ld.%02ld,%03X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
        		canData->timestamp / 100L,
				canData->timestamp % 100L,
				stdId,
				canData->canFrame.dataByte0,
				canData->canFrame.dataByte1,
				canData->canFrame.dataByte2,
				canData->canFrame.dataByte3,
				canData->canFrame.dataByte4,
				canData->canFrame.dataByte5,
				canData->canFrame.dataByte6,
				canData->canFrame.dataByte7);
	}
}
/* END CAN Frame Logging */
/* CAN Frame Printing */
void printCanData(flexcan_frame_t* canFrame) {
	if (canFrame->type == kFLEXCAN_FrameTypeData) {
		uint16_t stdId = CAN_STD_ID(canFrame->id);
        PRINTF("$CNDRV,%ld.%02ld,%08X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
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
	}
}
/* END CAN Frame Logging */

void txCanFrame() {
	/* Prepare Tx Frame for sending. */
	txFrame.format = kFLEXCAN_FrameFormatStandard;
	txFrame.type = kFLEXCAN_FrameTypeData;
	txFrame.id = FLEXCAN_ID_STD(0x14A);
	txFrame.length = 8;
	txFrame.dataWord0 = CAN_WORD0_DATA_BYTE_0(0x11) | CAN_WORD0_DATA_BYTE_1(0x22) | CAN_WORD0_DATA_BYTE_2(0x33) |
						CAN_WORD0_DATA_BYTE_3(0x44);
	txFrame.dataWord1 = CAN_WORD1_DATA_BYTE_4(0x55) | CAN_WORD1_DATA_BYTE_5(0x66) | CAN_WORD1_DATA_BYTE_6(0x77) |
						CAN_WORD1_DATA_BYTE_7(0x88);

	PRINTF("tx word0 = 0x%x\r\n", txFrame.dataWord0);
	PRINTF("tx word1 = 0x%x\r\n", txFrame.dataWord1);

	CAN_SendFrame(&txFrame);
}

/* END FLEXCAN module support */

/*
 * @brief   Application entry point.
 */
int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
  	/* Init FSL debug console. */
	BOARD_InitDebugConsole();

    PRINTF("Teensy Track Logger, v0.3\r\n");

    PRINTF("Initializing SD Card...\r\n");
    while (initSDcard() != kStatus_Success)
    {
    	PRINTF("Could not initialize SD card. Trying again...\r\n");
    	_delay(500U);
    }

    _delay(100U);
    PRINTF("Mounting SD filesystem...\r\n");
    while (mountSDfilesystem() != kStatus_Success)
    {
    	PRINTF("Could not mount SD filesystem.  Trying again...\r\n");
    	_delay(500U);
    }

    /* Move peripheral initialization after SD initialization */
    BOARD_InitBootPeripherals();

    PRINTF("Initializing the accelerometer module...\r\n");
    ACCEL_Init();

    PRINTF("Initializing the GPS module...\r\n");
    GPS_Init();

    openLogger("TRK");

    CAN_Enable();
    PRINTF("CAN module enabled.\r\n");

	status_t flexcanStatus = CAN_StartReceiving();
    PRINTF("CAN RxFifo Status: %d.\r\n", flexcanStatus);

    _delay(5000);

    bool isLogging = false;
	/* Force the counter to be placed into memory. */
    volatile static unsigned long i = 0 ;
    /* Enter an infinite loop, just incrementing a counter. */
    static int lastTimerTick = 0;
    while(1) {

        i++ ;

        if ((timerTicks != lastTimerTick) && ((timerTicks % 5) == 0)) {

        	lastTimerTick = timerTicks;

        	if ((timerTicks % 20) == 0) {

        		CAN_PrintStatus();

            	if ((timerTicks / 20L) & 1L) {
                	GPIO_PinWrite(GPIOC, 5U, 1);
//                	txCanFrame();
            	} else {
                	GPIO_PinWrite(GPIOC, 5U, 0);
            	}
            }

        	if ((timerTicks % 100) == 0) {
        		PRINTF("Time: %ld.%02ld  Accel: %u  CAN: %u  GPS: %u Logging: %u\r\n",
                		timerTicks / 100L,
        				timerTicks % 100L,
        				accelDataCount,
						canDataCount,
						gpsDataCount,
						isLogging);
        	}
        }

        // Do we need to log any CAN data
    	while (CAN_IsFrameAvailable()) {
    		can_data_t* frame = CAN_NextFrame();
    		if (isLogging) {
    			logCanData(frame);
//    			printCanData(frame);
    		}
    		canDataCount++;
    	}

        // Do we need to log any GPS data
        while (GPS_IsSentenceAvailable())
        {
        	char* nmeaSentence = GPS_NextSentence();
//        	GPS_PrintFifoState();
        	if (isLogging)
        	{
        		putLog(nmeaSentence);
        		PRINTF("%s\r", nmeaSentence);
        	}
        	gpsDataCount++;
        }

        // Do we need to log any accelerometer data
        while (ACCEL_IsDatapointAvailable()) {
        	accel_data_t* datapoint = ACCEL_NextDatapoint();
        	if (isLogging) {
        		appendLog("$AC001,%ld.%02ld,%d,%d,%d\n",
        				datapoint->timestamp / 100L,
        				datapoint->timestamp % 100L,
						datapoint->rawX,
						datapoint->rawY,
						datapoint->rawZ);
//        		PRINTF("$AC001,%ld.%02ld,%d,%d,%d\r\n",
//            		timerTicks / 100L,
//    				timerTicks % 100L,
//					datapoint->rawX,
//					datapoint->rawY,
//					datapoint->rawZ);
        	}
        	accelDataCount++;
        	// Start logging all data?
        	if (!isLogging
        			&& ((abs(datapoint->rawX - 501) > 8)
					|| (abs(datapoint->rawY - 514) > 8)))
        	{
        		isLogging = true;
        	}
        }

        ACCEL_Tick();
    }
    CAN_Disable();
    closeLogger();
    return 0 ;
}
