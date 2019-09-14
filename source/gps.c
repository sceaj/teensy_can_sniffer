/*
 * venusgps.c
 *
 *  Created on: Feb 21, 2019
 *      Author: Jeff Rosen
 */

#include <stdio.h>
#include "gps.h"
#include "fifobuffer.h"
#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"

#define GPS_UART UART0  /* Same as UART_1_PERIPHERAL */
#define GPS_MAX_NMEA_MESSAGE 80
#define GPS_FIFO_BUFFER_SIZE 64
#define GPS_UART_BUFFER_SIZE 2048
#define GPS_UART_IRQHandler UART0_RX_TX_IRQHandler  /* Same as UART_1_SERIAL_RX_TX_IRQHANDLER */

// From main module
void _delay(uint32_t delay_ms);

static const char* locosys_ack_format = "$PMTK001,%d,3*";
static const char* locosys_115200_baud = "$PMTK251,115200*1F\r\n";
static const char* locosys_config_nmea = "$PMTK314,0,1,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C\r\n";
static const char* locosys_5Hz_update = "$PMTK220,200*2C\r\n";

static uint8_t gps_uartBuffer[GPS_UART_BUFFER_SIZE + 1];
volatile uint16_t uartReadIndex;
volatile uint16_t uartWriteIndex;
uint8_t lastData = 0;
int resyncMode = false;

static char* gps_nmeaSentences[GPS_FIFO_BUFFER_SIZE];
volatile uint16_t nmeaReadIndex;
volatile uint16_t nmeaWriteIndex;

volatile uint16_t gpsUartIrqCount;

void GPS_UART_IRQHandler(void)
{
    uint8_t data;
    uint32_t statusFlags = UART_GetStatusFlags(GPS_UART);

    /* If new data arrived. */
    if (kUART_RxDataRegFullFlag & statusFlags)
    {
        data = UART_ReadByte(GPS_UART);

        /* If ring buffer is not full, add data to ring buffer. */
        if (!FIFO_BUFFER_FULL(uartReadIndex, uartWriteIndex, GPS_UART_BUFFER_SIZE))
        {
        	if ((data == '\n') && (lastData == '\r'))
        	{
        		gps_uartBuffer[uartWriteIndex - 1] = data;
        		gps_uartBuffer[uartWriteIndex] = 0;
        		if (!resyncMode) {
        			gps_nmeaSentences[nmeaWriteIndex] = (char*)(&gps_uartBuffer[uartReadIndex]);
        			nmeaWriteIndex = FIFO_BUFFER_ADVANCE(nmeaWriteIndex, GPS_FIFO_BUFFER_SIZE);
        		} else {
        			resyncMode = false;
        		}
                uartWriteIndex = FIFO_BUFFER_ADVANCE(uartWriteIndex, GPS_UART_BUFFER_SIZE);
                uartReadIndex = uartWriteIndex;

        	} else {
        		gps_uartBuffer[uartWriteIndex] = data;
                uartWriteIndex = FIFO_BUFFER_ADVANCE(uartWriteIndex, GPS_UART_BUFFER_SIZE);
                if (uartWriteIndex == 0)
                {
                	uartWriteIndex = GPS_UART_BUFFER_SIZE - uartReadIndex;
                	strcpy((char*)gps_uartBuffer, (char*)(&gps_uartBuffer[uartReadIndex]));
        			uartReadIndex = 0;
                }
        	}
        } else {
        	resyncMode = true;
        }

        lastData = data;
    }

    if (kUART_RxOverrunFlag & statusFlags)
    {
    	UART_ClearStatusFlags(GPS_UART, kUART_RxOverrunFlag);
    	resyncMode = true;
    }

    gpsUartIrqCount++;

    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

void GPS_PrintFifoState(void)
{
	PRINTF("UART FIFO: %u  %u  %d\r\n", uartReadIndex, uartWriteIndex, resyncMode);
	PRINTF("NMEA FIFO: %u %u\r\n", nmeaReadIndex, nmeaWriteIndex);
}

int GPS_LocosysCommand(const char* cmd, int cmdId)
{
	int result = 0;
	char expected[32];
	sprintf(expected, locosys_ack_format, cmdId);

	// Send command
	PRINTF("Sending: %s\r\n", cmd);
	UART_WriteBlocking(GPS_UART, (uint8_t*)cmd, strlen(cmd));
	// Wait up to 300ms for the ACK
	for (int i = 0; i < 6; i++) {
		_delay(100);
		if (GPS_IsSentenceAvailable()) {
			char* response = GPS_NextSentence();
			PRINTF("Response: %s", response);
			if (strncmp(response, expected, strlen(expected))) result = 1;
			break;
		}

		if (i == 5) {
			PRINTF("No Response\r\n");
			result = 2;
		}
	}

	return result;
}

void GPS_Init(void)
{
	// Change UART to 57600 baud
	UART_SetBaudRate(GPS_UART, 57600U, CLOCK_GetFreq(kCLOCK_CoreSysClk));

	// LOCOSYS: Change baud-rate to 115200
	GPS_LocosysCommand(locosys_115200_baud, 251);
	UART_SetBaudRate(GPS_UART, 115200U, CLOCK_GetFreq(kCLOCK_CoreSysClk));
	_delay(1000);

	// LOCOSYS: Configure NMEA sentence tx frequency
	GPS_LocosysCommand(locosys_config_nmea, 314);
	_delay(250);

	// LOCOSYS:
	GPS_LocosysCommand(locosys_5Hz_update, 220);
	_delay(250);
}

void GPS_Pause()
{
	// LOCOSYS: Enter low-power mode
}

void GPS_Restart()
{
	// LOCOSYS: Exit low-power mode
}

bool GPS_IsSentenceAvailable(void)
{
	return FIFO_AVL(nmeaReadIndex, nmeaWriteIndex);
}

char* GPS_NextSentence(void)
{
	if (FIFO_AVL(nmeaReadIndex, nmeaWriteIndex))
	{
		char* nextSentence = gps_nmeaSentences[nmeaReadIndex];
		nmeaReadIndex = FIFO_BUFFER_ADVANCE(nmeaReadIndex, GPS_FIFO_BUFFER_SIZE);
		return nextSentence;
	}

	return NULL;
}


