/*
 * ITM_Printf.c
 *
 *  Helper function to redirect Standard Out (from printf) to SWV ITM Port 0
 *				Created for ECE4240 Fall 2024 Lab 2
 *
 *	Note: This works specifically because __io_putchar is 'weakly' defined in the
			STM32 HAL.  This allows the programmer to override the function as desired.
 *
 *  Created on: Sep 20, 2024
 *      Author: David Stewart
 */

#include "ITM_Printf.h"

#define ITM_PORT 0  // Define the ITM port to use (Port 0)

int __io_putchar(int ch)
{
    if ((ITM->TCR & ITM_TCR_ITMENA_Msk) &&  // Check if ITM is enabled
        (ITM->TER & (1UL << ITM_PORT)))     // Check if Port 0 is enabled
    {
        while (ITM->PORT[ITM_PORT].u32 == 0);  // Wait until ITM is ready
        ITM->PORT[ITM_PORT].u8 = (uint8_t)ch;  // Send character to ITM Port 0
    }
    return ch;
}

