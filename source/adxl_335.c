/*
 * adxl_335.c
 *
 *  Created on: May 16, 2019
 *      Author: sceaj
 */

#include "accelerometer.h"
#include "fifobuffer.h"
#include "fsl_adc16.h"
#include "fsl_debug_console.h"
#include "peripherals.h"

/* ADXL335 data structures */
#define ACCEL_ADC16_CH_GRP 0
#define ACCEL_ADC16_CH_MASK 0x03
#define ACCEL_CONV_CPLT 0x10
#define ACCEL_CONV_IDLE 0x20

#define ACCEL_FIFO_BUFFER_SIZE 64

volatile uint8_t accelAxis = 0;
static accel_data_t accelFifo[ACCEL_FIFO_BUFFER_SIZE];
static int accelFifoRead = 0;
static int accelFifoWrite = 0;

/*
 * Accelerometer conversion complete Interrupt
 */
void ADC16_1_IRQHANDLER(void)
{
	uint16_t* pAdcValue = ((uint16_t*)&(accelFifo[accelFifoWrite].rawY)) + accelAxis;
	*pAdcValue = ADC16_GetChannelConversionValue(ADC16_1_PERIPHERAL, ACCEL_ADC16_CH_GRP);

	// Move to next axis and tell the main-loop to trigger the next conversion...
	++accelAxis;
	accelAxis %= 3;
	// Set conversion-complete bit
	accelAxis |= ACCEL_CONV_CPLT;
	// If we're back to axis 0, transistion to IDLE mode until TriggerConversion() is next called
	if (accelAxis == ACCEL_CONV_CPLT) {
		accelFifoWrite = FIFO_BUFFER_ADVANCE(accelFifoWrite, ACCEL_FIFO_BUFFER_SIZE);
	}
}
/* END Accelerometer Interrupt */

void ACCEL_Init(void)
{
    ADC16_DoAutoCalibration(ADC16_1_PERIPHERAL);
    accelAxis = ACCEL_CONV_IDLE;
}

void ACCEL_Tick(void)
{
    // Do we need to trigger the next accelerometer conversion?
    if (accelAxis & ACCEL_CONV_CPLT)
    {
    	accelAxis &= ACCEL_ADC16_CH_MASK;
    	if (accelAxis != 0) {
    		ADC16_SetChannelConfig(ADC16_1_PERIPHERAL, ACCEL_ADC16_CH_GRP, &ADC16_1_channelsConfig[accelAxis]);
    	} else {
    		accelAxis = ACCEL_CONV_IDLE;
    	}
    }
}

/*!
 * @brief Requests the ACCEL module to collect a datapoint.
 *
 */
void ACCEL_TriggerCollection(void)
{
	/* Start a new Accelerometer reading */
	if (accelAxis & ACCEL_CONV_IDLE)
	{
		accelAxis = 0;
		ADC16_SetChannelConfig(ADC16_1_PERIPHERAL, ACCEL_ADC16_CH_GRP, &ADC16_1_channelsConfig[accelAxis]);
	}
}


void ACCEL_PrintFifoState(void);

/*!
 * @brief Returns whether and NMEA sentences from the GPS are available.
 *
 * This function returns true if any sentences are available, otherwise false
 * is returned.
 */
int ACCEL_IsDatapointAvailable(void)
{
	return FIFO_AVL(accelFifoRead,accelFifoWrite);
}

/*!
 * @brief Returns the next NMEA sentence from the GPS if one is available.
 *
 * This function checks the GPS NMEA sentence FIFO to see if any sentences are available.
 * If there is at least 1 sentence available, a pointer to it is returned.  Otherwise a
 * NULL is returned.
 */
accel_data_t* ACCEL_NextDatapoint(void)
{
    if (FIFO_AVL(accelFifoRead,accelFifoWrite))
    {
    	accel_data_t* datapoint = &accelFifo[accelFifoRead];
    	accelFifoRead = FIFO_BUFFER_ADVANCE(accelFifoRead, ACCEL_FIFO_BUFFER_SIZE);
    	return datapoint;
    }

    return NULL;
}


