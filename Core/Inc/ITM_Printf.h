/*
 * ITM_Printf.h
 *
 *  Helper function to redirect Standard Out (from printf) to SWV ITM Port 0
 *              Created for ECE4240 Fall 2024 Lab 2
 *
 *	Note: This works specifically because __io_putchar is 'weakly' defined in the
 *			STM32 HAL.  This allows the programmer to override the function as desired.
 *
 *  Created on: Sep 20, 2024
 *      Author: David Stewart
 */

#ifndef INC_ITM_PRINTF_H_
#define INC_ITM_PRINTF_H_


#include "stm32f4xx.h"  // Include the required STM32 HAL header

#ifdef __cplusplus
extern "C" {
#endif

int __io_putchar(int ch);  // Declare the function for printf redirection

#ifdef __cplusplus
}
#endif

#endif /* INC_ITM_PRINTF_H_ */
