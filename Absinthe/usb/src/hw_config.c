/**
 ******************************************************************************
 * @file    hw_config.c
 * @author  MCD Application Team
 * @version V3.4.0
 * @date    29-June-2012
 * @brief   Hardware Configuration & Setup
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_istr.h"
#include "hw_config.h"
#include "usb_pwr.h"
#include "fifo_buffer.h"
#include "FreeRTOS.h"
#include "task.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USART_InitTypeDef USART_InitStructure;

t_fifo_buffer 	VCP_Tx_Buffer_Hnd[2];
uint8_t 		VCP1_Tx_Buffer[VCP_TX_DATA_SIZE];
uint8_t 		VCP2_Tx_Buffer[VCP_TX_DATA_SIZE];

t_fifo_buffer 	VCP_Rx_Buffer_Hnd[2];
uint8_t 		VCP1_Rx_Buffer[VCP_RX_DATA_SIZE];
uint8_t 		VCP2_Rx_Buffer[VCP_RX_DATA_SIZE];

/* Extern variables ----------------------------------------------------------*/
/* Extern functions ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
 * Function Name  : USB_Prepare
 * Description    : Configures Main system clocks & power
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void USB_Prepare(void)
{
	fifoBuf_init(&VCP_Rx_Buffer_Hnd[0], &VCP1_Rx_Buffer, VCP_RX_DATA_SIZE);
	fifoBuf_init(&VCP_Rx_Buffer_Hnd[1], &VCP2_Rx_Buffer, VCP_RX_DATA_SIZE);
	fifoBuf_init(&VCP_Tx_Buffer_Hnd[0], &VCP1_Tx_Buffer, VCP_TX_DATA_SIZE);
	fifoBuf_init(&VCP_Tx_Buffer_Hnd[1], &VCP2_Tx_Buffer, VCP_TX_DATA_SIZE);

	/* Disable the USB connection till initialization phase end */
	USB_Cable_Config(DISABLE);

	/* Delay on 50 ms using FreeRTOS function */
    vTaskDelay(50 / portTICK_RATE_MS);

	Set_USBClock();

	USB_Interrupts_Config();

	USB_Cable_Config(ENABLE);
}

/*******************************************************************************
 * Function Name  : Set_USBClock
 * Description    : Configures USB Clock input (48MHz)
 * Input          : None.
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
 * Function Name  : Enter_LowPowerMode
 * Description    : Power-off system clocks and power while entering suspend mode
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void Enter_LowPowerMode(void)
{
	/* Set the device state to suspend */
	bDeviceState = SUSPENDED;
}

/*******************************************************************************
 * Function Name  : Leave_LowPowerMode
 * Description    : Restores system clocks and power while exiting suspend mode
 * Input          : None.
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
 * Function Name  : USB_Interrupts_Config
 * Description    : Configures the USB interrupts
 * Input          : None.
 * Return         : None.
 *******************************************************************************/
void USB_Interrupts_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel 					 = USB_LP_CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		 = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd 				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
 * Function Name  : USB_Cable_Config
 * Description    : Software Connection/Disconnection of USB Cable
 * Input          : None.
 * Return         : Status
 *******************************************************************************/
void USB_Cable_Config(FunctionalState NewState)
{
	/* Using a "dirty" method to force a re-enumeration: */
	/* Force DPM (Pin PA12) low for ca. 10 mS before USB Tranceiver will be enabled */
	/* This overrules the external Pull-Up at PA12, and at least Windows & MacOS will enumerate again */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USB disconnect pin as Output open-drain */
	GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(USB_DISCONNECT_PORT, &GPIO_InitStructure);

	// To disable
	if (NewState == DISABLE)
	{
		/* Disable the Pull-Up*/
		GPIO_ResetBits(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
	}
	else
	{
		GPIO_SetBits(USB_DISCONNECT_PORT, USB_DISCONNECT_PIN);
	}
}

/*******************************************************************************
 * Function Name  :  USART_Config_Default.
 * Description    :  configure the EVAL_COM1 with default values.
 * Input          :  None.
 * Return         :  None.
 *******************************************************************************/
void USART_Config_Default(void)
{

}

/*******************************************************************************
 * Function Name  :  USART_Config.
 * Description    :  Configure the EVAL_COM1 according to the line coding structure.
 * Input          :  None.
 * Return         :  Configuration status
 TRUE : configuration done with success
 FALSE : configuration aborted.
 *******************************************************************************/
bool USART_Config(void)
{
	return (TRUE);
}

/*******************************************************************************
 * Function Name  : VCP_Get_Byte(void). User function
 * Description    : Return one byte received from USB VCP.
 * Input          : none.
 * Return         : Received byte.
 *******************************************************************************/
uint8_t vcp1GetByte(void)
{
	return fifoBuf_getByte(&VCP_Rx_Buffer_Hnd[0]);
}

uint8_t vcpGetByte(uint8_t chan)
{
	return fifoBuf_getByte(&VCP_Rx_Buffer_Hnd[chan]);
}

/*******************************************************************************
 * Function Name  : VCP_Has_Data(void). User function
 * Description    : Check USB VCP input buffer has a data.
 * Input          : none.
 * Return         : TRUE if buffer not empty.
 *******************************************************************************/
uint16_t vcp1HasData(void)
{
	return (fifoBuf_getUsed(&VCP_Rx_Buffer_Hnd[0]) == 0) ? FALSE : TRUE;
}

uint16_t vcpHasData(uint8_t chan)
{
	return (fifoBuf_getUsed(&VCP_Rx_Buffer_Hnd[chan]) == 0) ? FALSE : TRUE;
}

/*******************************************************************************
 * Function Name  : VCP_Send_Byte. User function
 * Description    : send data byte to USB.
 * Input          : data byte.
 * Return         : none.
 *******************************************************************************/
void vcp1SendByte(uint8_t data)
{
	fifoBuf_putByte(&VCP_Tx_Buffer_Hnd[0], data);
}

void vcpSendByte(uint8_t chan, uint8_t data)
{
	fifoBuf_putByte(&VCP_Tx_Buffer_Hnd[chan], data);
}

bool vcp1TransmitEmpty(void)
{
	return fifoBuf_getUsed(&VCP_Tx_Buffer_Hnd[0]) == 0;
}

bool vcpTransmitEmpty(uint8_t chan)
{
	return (fifoBuf_getUsed(&VCP_Tx_Buffer_Hnd[chan]) == 0);
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
		IntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2], 8);
		IntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
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
 * Description    : This function handles USB Low Priority interrupts
 *                  requests.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
	USB_Istr();
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
