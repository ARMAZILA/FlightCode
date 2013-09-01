/*
 * osd_core.c
 *
 *  Created on: 15.03.2013
 *      Author: avgorbi
 */

#include "main.h"

osdData_t osdData;

// We define pointers to each of these buffers.
uint8_t *draw_buffer_level;
uint8_t *draw_buffer_mask;
uint8_t *disp_buffer_level;
uint8_t *disp_buffer_mask;

xSemaphoreHandle osdSemaphore = NULL;

/**
 * swap_buffers: Swaps the two buffers. Contents in the display
 * buffer is seen on the output and the display buffer becomes
 * the new draw buffer.
 */
void swap_buffers()
{
        // While we could use XOR swap this is more reliable and
        // dependable and it's only called a few times per second.
        // Many compliers should optimise these to EXCH instructions.
        uint8_t *tmp;
        SWAP_BUFFS(tmp, disp_buffer_mask, draw_buffer_mask);
        SWAP_BUFFS(tmp, disp_buffer_level, draw_buffer_level);
}

/**
 * @brief  This function handles OSD_HSYNC External lines interrupt request.
 * @param  None
 * @retval None
 *
 * NOTE:	The signal OSD_HSYNC rise each start of new TV lines and go to fall at the end of
 * TV line. It's happen 31250 times per second and so this function is a subject for a tough
 * time optimization.
 */
void OSD_HSYNC_IRQHandler(void)
{
	uint16_t line;

	if (OSD_HSYNC_EXTI_PORT->IDR & OSD_HSYNC_EXTI_PIN)
	{
		//rising - start of line
		if (osdData.LineType == LINE_TYPE_GRAPHICS)
		{
			// Activate new OSD line
#if 0
			DMA1_Channel3->CCR |= DMA_CCR1_EN;
			DMA1_Channel5->CCR |= DMA_CCR1_EN;
#else
			/* Оптимизируем время между запуском двух каналов DMA */
			asm(
				"	push {r4, r5}		\n"		// Сохраняем в стеке регистры r4 и r5
				"	ldr	r3, =0x40020030	\n"		// Загружаем в r3 адрес регистра DMA1_Channel3->CCR
				"	ldr	r2, [r3, #0]	\n"		// Загружаем в r2 содержимое регистра DMA1_Channel3->CCR
				"	orr	r2, r2, #1		\n"		// Устанавливаем бит DMA_CCR1_EN
				"	ldr	r5, =0x40020058	\n"		// Загружаем в r5 адрес регистра DMA1_Channel5->CCR
				"	ldr	r4, [r5, #0]	\n"		// Загружаем в r4 содержимое регистра DMA1_Channel5->CCR
				"	orr	r4, r4, #1		\n"		// Устанавливаем бит DMA_CCR1_EN
				"	str	r4, [r5, #0]	\n"		// Сохраняем новое значение в DMA1_Channel5->CCR (разрешаем DMA1_Channel5)
				"	str	r2, [r3, #0]	\n"		// Сохраняем новое значение в DMA1_Channel3->CCR (разрешаем DMA1_Channel3)
				"	pop	{r4, r5}		\n"
			);
#endif
		}
	}
	else
	{
		//falling - blank field
		osdData.LineType = LINE_TYPE_BLANK; // Default case
		osdData.currentScanLine++;

		if ((osdData.currentScanLine >= GRAPHICS_LINE) && (osdData.currentScanLine < (GRAPHICS_LINE + GRAPHICS_HEIGHT)))
		{
			osdData.LineType = LINE_TYPE_GRAPHICS;
			osdData.activeScanLine = osdData.currentScanLine - GRAPHICS_LINE;
			line = osdData.activeScanLine * GRAPHICS_WIDTH;

			// Load new line

            // SPI1 DMA out BW
			DMA1_Channel3->CCR &= (uint16_t)(~DMA_CCR1_EN);				// Disable DMA1_Channel3
			DMA1_Channel3->CMAR = (uint32_t)&disp_buffer_level[line];
			DMA1_Channel3->CNDTR = BUFFER_LINE_LENGTH + 1;              // last byte must be zero always

			// SPI2 DMA out MASK
			DMA1_Channel5->CCR &= (uint16_t)(~DMA_CCR1_EN);				// Disable DMA1_Channel5
			DMA1_Channel5->CMAR = (uint32_t)&disp_buffer_mask[line];
			DMA1_Channel5->CNDTR = BUFFER_LINE_LENGTH + 1;
		}
	}

	/* Clear the HSYNC EXTI line pending bit */
	EXTI_ClearITPendingBit(OSD_HSYNC_EXTI_Line);
}

/**
 * @brief  This function handles VSYNC External interrupt request.
 * @param  None
 * @retval None
 *
 * PAL/SECAM have 625/50Hz and 288 active, NTSC 525 and 243 (242) active
 */
void OSD_VSYNC_IRQHandler(void)
{
	static uint8_t Vsync_update;
	static signed portBASE_TYPE xHigherPriorityTaskWoken;

	if (EXTI_GetITStatus(OSD_VSYNC_EXTI_LINE) != RESET)
	{
		xHigherPriorityTaskWoken = pdFALSE;

		osdData.maxScanLine = osdData.currentScanLine;
		osdData.currentScanLine = 0;
		osdData.PAL = osdData.maxScanLine > 300 ? true : false; // recheck mode
		osdData.Height = osdData.PAL ? GRAPHICS_HEIGHT_REAL : GRAPHICS_HEIGHT_REAL_NTSC;

		Vsync_update++;
		if (Vsync_update >= 2)
		{
			swap_buffers();
			Vsync_update = 0;
			xSemaphoreGiveFromISR(osdSemaphore, &xHigherPriorityTaskWoken);
		}

		//portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

		/* Clear the VSYNC EXTI line pending bit */
		EXTI_ClearITPendingBit(OSD_VSYNC_EXTI_LINE);
	}
}

static void osdIntConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configure OSD_HSYNC */
	RCC_APB2PeriphClockCmd(OSD_HSYNC_RCC_PORT, ENABLE);

	/* Initialize the GPIO pin */
	GPIO_InitStructure.GPIO_Pin   = OSD_HSYNC_EXTI_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(OSD_HSYNC_EXTI_PORT, &GPIO_InitStructure);

	// setup external interrupt
	GPIO_EXTILineConfig(OSD_HSYNC_EXTI_PORT_SRC, OSD_HSYNC_EXTI_PIN_SRC);

	EXTI_ClearITPendingBit(OSD_HSYNC_EXTI_Line);

	EXTI_InitStructure.EXTI_Line    = OSD_HSYNC_EXTI_Line;
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel 					 = OSD_HSYNC_IRQCHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		 = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure OSD_VSYNC */
	RCC_APB2PeriphClockCmd(OSD_VSYNC_RCC_PORT, ENABLE);

	/* Initialize the GPIO pin */
	GPIO_InitStructure.GPIO_Pin   = OSD_VSYNC_EXTI_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(OSD_VSYNC_EXTI_PORT, &GPIO_InitStructure);

	// setup external interrupt
	GPIO_EXTILineConfig(OSD_VSYNC_EXTI_PORT_SRC, OSD_VSYNC_EXTI_PIN_SRC);

	EXTI_InitStructure.EXTI_Line 	= OSD_VSYNC_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel 					 = OSD_VSYNC_IRQCHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		 = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd 				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
 * SPI1 - OSD_BW,   DMAtx - DMA1_Channel3
 * SPI2 - OSD_MASK, DMAtx - DMA1_Channel5
 */
static void osdSpiDmaConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStruct;

    // turn on peripherals. we're using SPI1, SPI2, DMA1 here
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // OSD SPI1 MOSI - PA7 Pin BW (Level)
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7; // SPI1 MOSI
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*
     * SPI1 тактируется от APB2 - PCLK2
     * APB2 тактируется от AHB без делителя (смотри system_stm32f10.c)
     *
     * SPI3 тактируется от APB1 - PCLK1
     * APB1 тактируется от AHB с делителем 2 (смотри system_stm32f10.c)
     */

    // OSD SPI1
    SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.SPI_Direction 	 = SPI_Direction_1Line_Tx;
    SPI_InitStruct.SPI_Mode 		 = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize 	 = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL 		 = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA 		 = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS 			 = SPI_NSS_Soft;
    SPI_InitStruct.SPI_FirstBit 	 = 0;
    SPI_InitStruct.SPI_CRCPolynomial = 0;
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	// 4,5 MHz
    SPI_Init(SPI1, &SPI_InitStruct);
    SPI_Cmd(SPI1, ENABLE);

    // OSD SPI1 tx DMA DMA1_Channel3
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Channel3);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
    DMA_InitStruct.DMA_MemoryBaseAddr 	  = (uint32_t)&disp_buffer_level;
    DMA_InitStruct.DMA_DIR 				  = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize 		  = GRAPHICS_WIDTH_REAL;
    DMA_InitStruct.DMA_PeripheralInc 	  = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc 		  = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize 	  = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode 			  = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority 		  = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_M2M 				  = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel3, &DMA_InitStruct);

    // OSD DMA for SPI1 setup
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

    // OSD SPI2 MOSI - PB15 Pin MASK
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    // OSD SPI2
    SPI_StructInit(&SPI_InitStruct);
    SPI_InitStruct.SPI_Direction	 = SPI_Direction_1Line_Tx;
    SPI_InitStruct.SPI_Mode 		 = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize 	 = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL 		 = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA 		 = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS 			 = SPI_NSS_Soft;
    SPI_InitStruct.SPI_FirstBit 	 = 0;
    SPI_InitStruct.SPI_CRCPolynomial = 0;
    //SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    //SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; 	// ~800 pixels on x axis
    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; 	// 400 pixels on x axis
    //SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; 	// ~200 pixels on x axis
    SPI_Init(SPI2, &SPI_InitStruct);
    SPI_Cmd(SPI2, ENABLE);

    // OSD SPI2 tx DMA DMA1_Channel5
    DMA_StructInit(&DMA_InitStruct);
    DMA_DeInit(DMA1_Channel5);
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
    DMA_InitStruct.DMA_MemoryBaseAddr 	  = (uint32_t)&disp_buffer_mask;
    DMA_InitStruct.DMA_DIR 				  = DMA_DIR_PeripheralDST;
    DMA_InitStruct.DMA_BufferSize 		  = GRAPHICS_WIDTH_REAL;
    DMA_InitStruct.DMA_PeripheralInc 	  = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc 		  = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStruct.DMA_MemoryDataSize 	  = DMA_MemoryDataSize_Byte;
    DMA_InitStruct.DMA_Mode 			  = DMA_Mode_Normal;
    DMA_InitStruct.DMA_Priority 		  = DMA_Priority_VeryHigh;
    DMA_InitStruct.DMA_M2M 				  = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStruct);

    // OSD DMA for SPI2
    SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
}

void osdInit(void)
{
	// Create semaphore for OSD frame drawing syncronization
	vSemaphoreCreateBinary(osdSemaphore);

	/* Configure SPI and DMA perephirial */
	osdSpiDmaConfig();

	/* Configure the Video Line interrupt */
	osdIntConfig();

	draw_buffer_level = osdData.buffer0_level;
	draw_buffer_mask  = osdData.buffer0_mask;
    disp_buffer_level = osdData.buffer1_level;
    disp_buffer_mask  = osdData.buffer1_mask;

	osdData.buffer_padding = 0;
	osdData.currentScanLine = 0;
	osdData.activeScanLine  = 0;
	osdData.LineType = LINE_TYPE_BLANK;
}
