/**
 ******************************************************************************
  * @file    hw_config.h
  * @author  avgorbi
  * @version V1.0.0
  * @date    04-May-2013
  * @brief   Hardware Configuration & Setup
 ******************************************************************************
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h"
#include "dfu_mal.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "usb_istr.h"

/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

/*******************************************************************************
 * Function Name  : Set_System.
 * Description    : Configures Main system clocks & power.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_System(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/*!< At this stage the microcontroller clock setting is already configured,
	 this is done through SystemInit() function which is called from startup
	 file (startup_stm32f10x_xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32f10x.c file
	 */

	/* Unlock the internal flash */
	FLASH_Unlock();

	/* Enable "DISCONNECT" GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);

	/* Configure USB pull-up */
	GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(USB_DISCONNECT_PORT, &GPIO_InitStructure);

	/* Disable the USB connection till initialization phase end */
	USB_Cable_Config(DISABLE);
	bl_delay_ms(50);

	/* Init the media interface */
	MAL_Init();

	USB_Cable_Config(ENABLE);
}

/*******************************************************************************
 * Function Name  : Set_USBClock.
 * Description    : Configures USB Clock input (48MHz).
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Set_USBClock(void)
{
	/* Select USBCLK source */
	RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

	/* Enable the USB clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
}

/*******************************************************************************
 * Function Name  : Enter_LowPowerMode.
 * Description    : Power-off system clocks and power while entering suspend mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Enter_LowPowerMode(void)
{
	/* Set the device state to suspend */
	bDeviceState = SUSPENDED;
}

/*******************************************************************************
 * Function Name  : Leave_LowPowerMode.
 * Description    : Restores system clocks and power while exiting suspend mode.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Leave_LowPowerMode(void)
{
	DEVICE_INFO *pInfo = &Device_Info;

	/* Set the device state to the correct state */
	if (pInfo->Current_Configuration != 0)
	{
		/* Device configured */
		bDeviceState = CONFIGURED;
	}
	else
	{
		bDeviceState = ATTACHED;
	}
}

/*******************************************************************************
 * Function Name  : USB_Cable_Config.
 * Description    : Software Connection/Disconnection of USB Cable.
 * Input          : NewState: new state.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void USB_Cable_Config(FunctionalState NewState)
{
	if (NewState == DISABLE)
	{
		GPIO_ResetBits(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
	}
	else
	{
		GPIO_SetBits(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
	}
}

/*******************************************************************************
 * Function Name  : DFU_Key_Present.
 * Description    : Reads the DFU selector key to enter DFU Mode.
 * Input          : None.
 * Output         : None.
 * Return         : Status. 1 - if key present, enter to DFU mode
 * 							0 - no key, go to application
 *******************************************************************************/
uint8_t DFU_Key_Present(void)
{
	/* Read pin PB10 (I2C2_SCL) */
	return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) ? FALSE : TRUE);
}

/*******************************************************************************
 * Function Name  : USB_Interrupts_Config.
 * Description    : Configures the USB interrupts.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void USB_Interrupts_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * Function Name  : Reset_Device.
 * Description    : Reset the device.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Reset_Device(void)
{
	USB_Cable_Config(DISABLE);
	NVIC_SystemReset();
}

/*******************************************************************************
 * Function Name  : Get_SerialNum.
 * Description    : Create the serial number string descriptor.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void Get_SerialNum(void)
{
	uint32_t Device_Serial0, Device_Serial1, Device_Serial2;

	Device_Serial0 = *(__IO uint32_t*) (0x1FFFF7E8);
	Device_Serial1 = *(__IO uint32_t*) (0x1FFFF7EC);
	Device_Serial2 = *(__IO uint32_t*) (0x1FFFF7F0);

	Device_Serial0 += Device_Serial2;

	if (Device_Serial0 != 0)
	{
		IntToUnicode(Device_Serial0, &DFU_StringSerial[2], 8);
		IntToUnicode(Device_Serial1, &DFU_StringSerial[18], 4);
	}
}

/*******************************************************************************
 * Function Name  : HexToChar.
 * Description    : Convert Hex 32Bits value into char.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
	uint8_t idx = 0;

	for (idx = 0; idx < len; idx++)
	{
		if (((value >> 28)) < 0xA)
		{
			pbuf[2 * idx] = (value >> 28) + '0';
		}
		else
		{
			pbuf[2 * idx] = (value >> 28) + 'A' - 10;
		}

		value = value << 4;

		pbuf[2 * idx + 1] = 0;
	}
}

/*******************************************************************************
 * Function Name  : USB_IRQHandler
 * Description    : This function handles USB Low Priority interrupts requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	USB_Istr();
}

/**
 * @brief  Inserts a delay time.
 * @param  nCount: specifies the delay time length.
 * @retval None
 */
void bl_delay(__IO uint32_t nCount)
{
	/* Decrement nCount value */
	while (nCount != 0)
	{
		nCount--;
	}
}

/**
 * @brief  Inserts a delay time in miliseconds.
 * @param  nCount: specifies the delay time length.
 * @retval None
 */
void bl_delay_ms(__IO uint32_t nCount)
{
	/* Decrement nCount value */
	while (nCount != 0)
	{
		nCount--;
		bl_delay(7200);
	}
}

/**
  * @brief  Checks if the Backup DR1 register contains a magic number.
  * @param  None.
  * @retval  - TRUE: Backup DR1 register contains a magic number - main
  * 		   programm request to start bootloader.
  *          - FALSE: otherwise.
  */
uint8_t CheckBackupReg(void)
{
	if (BKP_ReadBackupRegister(BKP_DR1) == MAGIC_NUMBER)
	{
		/* Clear magic number, in any case */
		BKP_WriteBackupRegister(BKP_DR1, 0);
		return (TRUE);
	}

	return FALSE;
}
