/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_flexcan.h"
#include "fsl_clock.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Definition of peripheral ID */
#define FLEXCAN_1_PERIPHERAL CAN0
/* Definition of the clock source frequency */
#define FLEXCAN_1_CLOCK_SOURCE 16000000UL

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern const flexcan_config_t FlexCAN_1_config;
/* Message buffer 9 configuration structure */
extern const flexcan_rx_mb_config_t FlexCAN_1_rx_mb_config_9;
extern flexcan_rx_fifo_config_t FlexCAN_1_rx_fifo_config;
extern void * g_flexcanRxFilters;
extern flexcan_handle_t FlexCAN_1_handle;

/***********************************************************************************************************************
 * Callback functions
 **********************************************************************************************************************/
/* FlexCAN transfer callback function for the FlexCAN_1 component (init. function BOARD_InitPeripherals)*/
extern void flexcanCallback(CAN_Type *, flexcan_handle_t *, status_t , uint32_t , void *);

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
