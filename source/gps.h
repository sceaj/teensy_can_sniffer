/*
 * gps.h
 *
 *  Created on: May 15, 2019
 *      Author: sceaj
 */

#ifndef GPS_H_
#define GPS_H_

#include "stdbool.h"

#define STOP false
#define START true

void GPS_PrintFifoState(void);

void GPS_Init(void);

void GPS_Pause(void);

void GPS_Restart(void);

/*!
 * @brief Returns whether and NMEA sentences from the GPS are available.
 *
 * This function returns true if any sentences are available, otherwise false
 * is returned.
 */
bool GPS_IsSentenceAvailable(void);

/*!
 * @brief Returns the next NMEA sentence from the GPS if one is available.
 *
 * This function checks the GPS NMEA sentence FIFO to see if any sentences are available.
 * If there is at least 1 sentence available, a pointer to it is returned.  Otherwise a
 * NULL is returned.
 */
char* GPS_NextSentence(void);

#endif /* GPS_H_ */
