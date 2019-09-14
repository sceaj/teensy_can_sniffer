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
 * @file    board.c
 * @brief   Board initialization file.
 */
 
/* This is a template for board specific configuration created by MCUXpresso IDE Project Wizard.*/

#include <stdint.h>
#include "board.h"

#include "fsl_clock.h"
#include "fsl_debug_console.h"
#include "serial_manager.h"

#define BOARD_DEBUG_USBCDC_BAUDRATE  115200
#define BOARD_DEBUG_USBCDC_INSTANCE  0
#define BOARD_DEBUG_USBCDC_PORT_TYPE kSerialPort_UsbCdc
#define BOARD_DEBUG_USBCDC_CLK_FREQ MCG_INTERNAL_IRC_48M

#define BOARD_DEBUG_UART_BAUDRATE    115200
#define BOARD_DEBUG_UART_INSTANCE    2
#define BOARD_DEBUG_UART_PORT_TYPE   kSerialPort_Uart
#define BOARD_DEBUG_UART_CLK_FREQ    CLOCK_GetFreq(UART2_CLK_SRC)

#if (defined(SERIAL_PORT_TYPE_UART) && (SERIAL_PORT_TYPE_UART > 0U))
#define BOARD_DEBUG_BAUDRATE         BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_INSTANCE         BOARD_DEBUG_UART_INSTANCE
#define BOARD_DEBUG_PORT_TYPE        BOARD_DEBUG_UART_PORT_TYPE
#define BOARD_DEBUG_PORT_CLK_FREQ    BOARD_DEBUG_UART_CLK_FREQ
#else
#define BOARD_DEBUG_BAUDRATE         BOARD_DEBUG_USBCDC_BAUDRATE
#define BOARD_DEBUG_INSTANCE         BOARD_DEBUG_USBCDC_INSTANCE
#define BOARD_DEBUG_PORT_TYPE        BOARD_DEBUG_USBCDC_PORT_TYPE
#define BOARD_DEBUG_PORT_CLK_FREQ    BOARD_DEBUG_USBCDC_CLK_FREQ
#endif
/**
 * @brief Set up and initialize all required blocks and functions related to the board hardware.
 */
void BOARD_InitDebugConsole(void) {
	/* The user initialization should be placed here */
    DbgConsole_Init(BOARD_DEBUG_INSTANCE, BOARD_DEBUG_BAUDRATE, BOARD_DEBUG_PORT_TYPE, BOARD_DEBUG_PORT_CLK_FREQ);
}
