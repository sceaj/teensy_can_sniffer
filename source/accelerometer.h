/*
 * accelerometer.h
 *
 *  Created on: May 16, 2019
 *      Author: sceaj
 */

#ifndef ACCEL_H_
#define ACCEL_H_

#include <stdint.h>

typedef struct {
	int16_t  rawY;  // Pin A8 (PTC8)
	int16_t  rawX;  // Pin D7 (PTC9)
	int16_t  rawZ;  // Pin C7 (PTC10)
} accel_data_t;

/*!
 * @brief Gives ACCEL module a chance to perform housekeeping and background processing.
 *
 * This method should be called at least 4 times as frequently as ACCEL_TriggerCollection.
 * It is lightweight, so calling much more frequently should not cause any problems.
 */
void ACCEL_Tick(void);

/*!
 * @brief Performs any one-time initialization of the module.
 *
 * This function should be called before any other functions in this module.
 */
void ACCEL_Init(void);

/*!
 * @brief Requests the ACCEL module to collect a datapoint.
 *
 */
void ACCEL_TriggerCollection(void);


void ACCEL_PrintFifoState(void);

/*!
 * @brief Returns whether and NMEA sentences from the GPS are available.
 *
 * This function returns true if any sentences are available, otherwise false
 * is returned.
 */
int ACCEL_IsDatapointAvailable(void);

/*!
 * @brief Returns the next NMEA sentence from the GPS if one is available.
 *
 * This function checks the GPS NMEA sentence FIFO to see if any sentences are available.
 * If there is at least 1 sentence available, a pointer to it is returned.  Otherwise a
 * NULL is returned.
 */
accel_data_t* ACCEL_NextDatapoint(void);

#endif /* GPS_H_ */
