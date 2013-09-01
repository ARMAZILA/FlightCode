/**
 ******************************************************************************
 * @file    usb_endp.c
 * @author  MCD Application Team
 * @version V3.4.0
 * @date    29-June-2012
 * @brief   Endpoint routines
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
#include "usb_desc.h"
#include "usb_mem.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "fifo_buffer.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL             5

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

extern t_fifo_buffer VCP_Tx_Buffer_Hnd[2];
extern t_fifo_buffer VCP_Rx_Buffer_Hnd[2];
uint8_t VCP1_Tx_State;
uint8_t VCP2_Tx_State;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
 * Function Name  : EP1_IN_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void EP1_IN_Callback(void)
{
	uint16_t USB_Tx_length;
	uint8_t  USB_Tx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];

	if (VCP1_Tx_State == 1)
	{
		USB_Tx_length = fifoBuf_getUsed(&VCP_Tx_Buffer_Hnd[0]);
		if (USB_Tx_length == 0)
		{
			VCP1_Tx_State = 0;
		}
		else
		{
			if (USB_Tx_length > VIRTUAL_COM_PORT_DATA_SIZE)
			{
				USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
			}

			fifoBuf_getData(&VCP_Tx_Buffer_Hnd[0], USB_Tx_Buffer, USB_Tx_length);

			UserToPMABufferCopy(USB_Tx_Buffer, ENDP1_TXADDR, USB_Tx_length);
			SetEPTxCount(ENDP1, USB_Tx_length);
			SetEPTxValid(ENDP1);
		}
	}
}

void EP4_IN_Callback(void)
{
	uint16_t USB_Tx_length;
	uint8_t  USB_Tx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];

	if (VCP2_Tx_State == 1)
	{
		USB_Tx_length = fifoBuf_getUsed(&VCP_Tx_Buffer_Hnd[1]);
		if (USB_Tx_length == 0)
		{
			VCP2_Tx_State = 0;
		}
		else
		{
			if (USB_Tx_length > VIRTUAL_COM_PORT_DATA_SIZE)
			{
				USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
			}

			fifoBuf_getData(&VCP_Tx_Buffer_Hnd[1], USB_Tx_Buffer, USB_Tx_length);

			UserToPMABufferCopy(USB_Tx_Buffer, ENDP4_TXADDR, USB_Tx_length);
			SetEPTxCount(ENDP4, USB_Tx_length);
			SetEPTxValid(ENDP4);
		}
	}
}

/*******************************************************************************
 * Function Name  : Handle_USBAsynchXfer.
 * Description    : send data to USB.
 * Input          : None.
 * Return         : none.
 *******************************************************************************/
void Handle_USBAsynchXfer(void)
{
	uint16_t USB_Tx_length;
	uint8_t  USB_Tx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];

	if (VCP1_Tx_State != 1)
	{
		USB_Tx_length = fifoBuf_getUsed(&VCP_Tx_Buffer_Hnd[0]);
		if (USB_Tx_length == 0)
		{
			VCP1_Tx_State = 0;
		}
		else
		{
			if (USB_Tx_length > VIRTUAL_COM_PORT_DATA_SIZE)
			{
				USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
			}

			fifoBuf_getData(&VCP_Tx_Buffer_Hnd[0], USB_Tx_Buffer, USB_Tx_length);
			VCP1_Tx_State = 1;

			UserToPMABufferCopy(USB_Tx_Buffer, ENDP1_TXADDR, USB_Tx_length);
			SetEPTxCount(ENDP1, USB_Tx_length);
			SetEPTxValid(ENDP1);
		}
	}

	if (VCP2_Tx_State != 1)
	{
		USB_Tx_length = fifoBuf_getUsed(&VCP_Tx_Buffer_Hnd[1]);
		if (USB_Tx_length == 0)
		{
			VCP2_Tx_State = 0;
		}
		else
		{
			if (USB_Tx_length > VIRTUAL_COM_PORT_DATA_SIZE)
			{
				USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
			}

			fifoBuf_getData(&VCP_Tx_Buffer_Hnd[1], USB_Tx_Buffer, USB_Tx_length);
			VCP2_Tx_State = 1;

			UserToPMABufferCopy(USB_Tx_Buffer, ENDP4_TXADDR, USB_Tx_length);
			SetEPTxCount(ENDP4, USB_Tx_length);
			SetEPTxValid(ENDP4);
		}
	}
}

/*******************************************************************************
 * Function Name  : SOF_Callback / INTR_SOFINTR_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void SOF_Callback(void)
{
	static uint32_t FrameCount = 0;

	if (bDeviceState == CONFIGURED)
	{
		if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
		{
			/* Reset the frame counter */
			FrameCount = 0;

			/* Check the data to be sent through IN pipe */
			Handle_USBAsynchXfer();
		}
	}
}

/*******************************************************************************
 * Function Name  : EP3_OUT_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void EP3_OUT_Callback(void)
{
	uint16_t USB_Rx_Cnt;
	uint8_t	 USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];

	/* Get the received data buffer and update the counter */
	USB_Rx_Cnt = USB_SIL_Read(EP3_OUT, USB_Rx_Buffer);

	/* USB data will be immediately processed, this allow next USB traffic being
	 NAKed till the end of the USART Xfer */

	fifoBuf_putData(&VCP_Rx_Buffer_Hnd[0], USB_Rx_Buffer, USB_Rx_Cnt);

	/* Enable the receive of data on EP3 */
	SetEPRxValid(ENDP3);
}

/*******************************************************************************
 * Function Name  : EP6_OUT_Callback
 * Description    :
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
void EP6_OUT_Callback(void)
{
	uint16_t USB_Rx_Cnt;
	uint8_t	 USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];

	/* Get the received data buffer and update the counter */
	USB_Rx_Cnt = USB_SIL_Read(EP6_OUT, USB_Rx_Buffer);

	fifoBuf_putData(&VCP_Rx_Buffer_Hnd[1], USB_Rx_Buffer, USB_Rx_Cnt);

	/* Enable the receive of data on EP6 */
	SetEPRxValid(ENDP6);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

