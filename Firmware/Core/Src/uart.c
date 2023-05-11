/* C Library Includes */

/* TI Library Includes */
#include <ti/drivers/UART.h>

/* Personal Includes */
#include "uart.h"
#include "utils.h"

/* Private Variables */
UART_Params uart_params;
UART_Handle* UartHandle

uint8_t init_uart(UART_Handle* uHandle) {

    // Set UART parameters
    UART_Params_init(&uart_params);
    uart_params.baudRate = 115200;
    uart_params.writeDataMode = UART_DATA_BINARY;
    uart_params.readDataMode = UART_DATA_BINARY;
    uart_params.readReturnMode = UART_RETURN_FULL;
    uart_params.readEcho = UART_ECHO_OFF;

    // Open UART0
    UartHandle = uHandle;
    *UartHandle = UART_open(Board_UART0, &uart_params);

    // Check for errors
    if (uHandle == NULL) {
        return FALSE;
    }

    return TRUE;
}

void uart_transmit_data(uint8_t* data, uint16_t numBytes) {
    UART_write(*uHandle, data, numBytes);
}