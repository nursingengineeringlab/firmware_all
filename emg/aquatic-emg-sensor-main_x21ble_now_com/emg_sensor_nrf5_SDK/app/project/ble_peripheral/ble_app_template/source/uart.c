/**
 * @file       uarte.c
 * @version    1.0.0
 * @date       2022-02-15
 * @author     Akshat
 * @brief      System module for UART
 * @note       None
 * @example    None
 */

/* Includes ----------------------------------------------------------- */
#include "uart.h"

/* Private defines ---------------------------------------------------- */
/* Private enumerate/structure ---------------------------------------- */
/* Private macros ----------------------------------------------------- */
/* Public variables --------------------------------------------------- */
/* Private variables -------------------------------------------------- */
/* Public defines ---------------------------------------------------- */
#define UART_TX_BUFF_SIZE 128 // TX buffer size
#define UART_RX_BUFF_SIZE 128 // RX Buffer size
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

/* Private function prototypes ---------------------------------------- */
/* Function definitions ----------------------------------------------- */
void uart_error_handle(app_uart_evt_t * p_event)
{
    /*
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
    */
}

void user_uart_init(void) {
    uint32_t err_code_uart;

    const app_uart_comm_params_t com_params = // struct to hold the uart configurations
    {
        RX_PIN_NUMBER, 
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        false, //UART_HWFC, 
        false,
        NRF_UART_BAUDRATE_115200 
    };

    APP_UART_FIFO_INIT(&com_params, UART_RX_BUFF_SIZE, UART_TX_BUFF_SIZE, uart_error_handle, APP_IRQ_PRIORITY_LOWEST, err_code_uart);
}

/* Private function definitions ---------------------------------------- */
/* End of file -------------------------------------------------------- */