#include "main.h"

/*
	UART1 - Bloetooth, XBee, Radio modem, ... (MSP / MAVLink protocol / Simulator)
	UART5 - FrSky telemetry, S.Bus
 	UART3/4 - GPS
*/

/* UART config structure */
struct uart_cfg_s {
	USART_TypeDef 	   *regs;				/* UART port */
	uint32_t 			remap;				/* GPIO_Remap_* */
	GPIO_TypeDef 	   *rx_gpio_port;
	uint16_t			rx_gpio_pin;
	GPIO_TypeDef 	   *tx_gpio_port;
	uint16_t			tx_gpio_pin;
	uint8_t 			NVIC_IRQChannel;
};

static const struct uart_cfg_s uart_cfg[3] =
{
	{	// ---------- Serial port id 0, USART1 ---------------------
		.regs 			 = USART1,
		.tx_gpio_port 	 = GPIOA,
		.tx_gpio_pin 	 = GPIO_Pin_9,
		.rx_gpio_port 	 = GPIOA,
		.rx_gpio_pin 	 = GPIO_Pin_10,
		.NVIC_IRQChannel = USART1_IRQn,
		.remap 			 = 0,
	},
	{	// ---------- Serial port id 1, USART3 ---------------------
		.regs 			 = USART3,
		.tx_gpio_port 	 = GPIOC,
		.tx_gpio_pin 	 = GPIO_Pin_10,
		.rx_gpio_port 	 = GPIOC,
		.rx_gpio_pin 	 = GPIO_Pin_11,
		.NVIC_IRQChannel = USART3_IRQn,
		.remap 			 = GPIO_PartialRemap_USART3,
	},
	{	// ---------- Serial port id 2, USART5 ---------------------
		.regs 			 = UART5,
		.tx_gpio_port 	 = GPIOC,
		.tx_gpio_pin 	 = GPIO_Pin_12,
		.rx_gpio_port 	 = GPIOD,
		.rx_gpio_pin 	 = GPIO_Pin_2,
		.NVIC_IRQChannel = UART5_IRQn,
		.remap			 = 0,
	},
};

#define UART_BUFFER_SIZE    128

// Receive buffer, circular
volatile uint8_t 		txBuffer[3][UART_BUFFER_SIZE];
uint32_t 				txBufferTail[3] = { 0, 0, 0 };
uint32_t 				txBufferHead[3] = { 0, 0, 0 };
uartReceiveCallbackPtr 	uartCallback[3] = { NULL, NULL, NULL };

void UARTx_IRQHandler(uint8_t uart_id)
{
	USART_TypeDef* USARTx = uart_cfg[uart_id].regs;
    uint16_t SR = USARTx->SR;

    if (SR & USART_IT_RXNE)
    {
        if (uartCallback[uart_id]) uartCallback[uart_id](USART_ReceiveData(USARTx));
    }
    if (SR & USART_FLAG_TXE)
    {
        if (txBufferTail[uart_id] != txBufferHead[uart_id])
        {
        	USARTx->DR = txBuffer[uart_id][txBufferTail[uart_id]];
            txBufferTail[uart_id] = (txBufferTail[uart_id] + 1) % UART_BUFFER_SIZE;
        }
        else
        {
            USART_ITConfig(USARTx, USART_IT_TXE, DISABLE);
        }
    }
}

void USART1_IRQHandler(void)
{
	UARTx_IRQHandler(SERIAL_UART1);
}

void USART3_IRQHandler(void)
{
	UARTx_IRQHandler(SERIAL_UART3);
}

void UART5_IRQHandler(void)
{
	UARTx_IRQHandler(SERIAL_UART5);
}

static void uartOpen(USART_TypeDef* USARTx, uint32_t speed)
{
    USART_InitTypeDef USART_InitStructure;

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate 			  = speed;
    USART_InitStructure.USART_WordLength 		  = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits 			  = USART_StopBits_1;
    USART_InitStructure.USART_Parity 			  = USART_Parity_No;
    USART_InitStructure.USART_Mode 				  = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USARTx, &USART_InitStructure);

    USART_Cmd(USARTx, ENABLE);
}

void uartChangeBaud(uint8_t uart_id, uint32_t speed)
{
    uartOpen(uart_cfg[uart_id].regs, speed);
}

void uartInit(uint8_t uart_id, uint32_t speed, uartReceiveCallbackPtr func)
{
    USART_TypeDef* USARTx = uart_cfg[uart_id].regs;

	/* Enable USART clock */
	switch ((uint32_t)USARTx) {
	case (uint32_t)USART1:
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		break;
	case (uint32_t)USART2:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		break;
	case (uint32_t)USART3:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		break;
	case (uint32_t)UART4:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		break;
	case (uint32_t)UART5:
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		break;
	}

    /* Configure USART Interrupts */
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel 					 = uart_cfg[uart_id].NVIC_IRQChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority		 = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd 				 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	/* Initialize the USART Rx and Tx pins */
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin 	= uart_cfg[uart_id].tx_gpio_pin;
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed 	= GPIO_Speed_2MHz;
    GPIO_Init(uart_cfg[uart_id].tx_gpio_port, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin 	= uart_cfg[uart_id].rx_gpio_pin;
    GPIO_InitStructure.GPIO_Mode 	= GPIO_Mode_IPU;
    GPIO_Init(uart_cfg[uart_id].rx_gpio_port, &GPIO_InitStructure);

    /* Enable the USART Pins Software Remapping */
	if (uart_cfg[uart_id].remap) {
		GPIO_PinRemapConfig(uart_cfg[uart_id].remap, ENABLE);
	}

	/* Enable USART */
	uartOpen(USARTx, speed);

	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);

    uartCallback[uart_id] = func;
}

bool uartTransmitEmpty(uint8_t uart_id)
{
    return txBufferTail[uart_id] == txBufferHead[uart_id];
}

void uartWrite(uint8_t uart_id, uint8_t ch)
{
    txBuffer[uart_id][txBufferHead[uart_id]] = ch;
    txBufferHead[uart_id] = (txBufferHead[uart_id] + 1) % UART_BUFFER_SIZE;

    USART_ITConfig(uart_cfg[uart_id].regs, USART_IT_TXE, ENABLE);
}

void uartPrint(uint8_t uart_id, char *str)
{
    while (*str)
        uartWrite(uart_id, *(str++));
}
