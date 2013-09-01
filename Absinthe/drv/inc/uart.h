#ifndef __UART_H
#define __UART_H

// Serial port device ID
enum {
    SERIAL_UART1 = 0,
    SERIAL_UART3 = 1,
    SERIAL_UART5 = 2,
};

typedef void (* uartReceiveCallbackPtr)(uint16_t data);     // used by uart2 driver to return frames to app

void uartInit(uint8_t uart_id, uint32_t speed, uartReceiveCallbackPtr func);
bool uartTransmitEmpty(uint8_t uart_id);
void uartWrite(uint8_t uart_id, uint8_t ch);
void uartPrint(uint8_t uart_id, char *str);
void uartChangeBaud(uint8_t uart_id, uint32_t speed);


/*
	UART helper functions template

#include "fifo_buffer.h"

#define	RX_BUFFER_SIZE	64
static t_fifo_buffer	Rx_Buffer_Hnd;
static uint8_t	 		Rx_Buffer[X_RX_BUFFER_SIZE];

static void xCallback(uint16_t data)
{
	fifoBuf_putByte(&Rx_Buffer_Hnd, data);
}

static uint16_t xHasData(void)
{
	return (fifoBuf_getUsed(&Rx_Buffer_Hnd) == 0) ? false : true;
}

static uint8_t xRead(void)
{
    return fifoBuf_getByte(&Rx_Buffer_Hnd);
}

static void xWrite(uint8_t data)
{
	uartWrite(UARTx, data);
}

*/
#endif
