/**
 * @file       uarte.c
 * @version    1.0.0
 * @date       2022-02-15
 * @author     Akshat
 * @brief      System module for non-blocking UART
 * @note       None
 * @example    None
 */

#ifndef __UART_PRINTF_H
#define __UART_PRINTF_H

/* Includes ----------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"
#include "nrf_uart.h"

/* Public defines ----------------------------------------------------- */
/* Public enumerate/structure ----------------------------------------- */
/* Public macros ------------------------------------------------------ */
/* Public variables --------------------------------------------------- */
/* Public function prototypes ----------------------------------------- */
void uart_error_handle(app_uart_evt_t * p_event); 
void user_uart_init(void); 

#endif

/* End of file -------------------------------------------------------- */

