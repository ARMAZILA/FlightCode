/**
  ******************************************************************************
  * @file    hw_config.h
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "usb_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

#define USB_DISCONNECT_PORT                 GPIOA
#define USB_DISCONNECT_PIN                  GPIO_Pin_12
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA

#define VCP_TX_DATA_SIZE					256
#define VCP_RX_DATA_SIZE					128

/* Exported functions ------------------------------------------------------- */
void USB_Prepare(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Interrupts_Config(void);
void USB_Cable_Config (FunctionalState NewState);
void USART_Config_Default(void);
bool USART_Config(void);

uint8_t vcp1GetByte(void);
uint16_t vcp1HasData(void);
bool vcp1TransmitEmpty(void);
void vcp1SendByte(uint8_t data);

uint8_t vcpGetByte(uint8_t chan);
uint16_t vcpHasData(uint8_t chan);
void vcpSendByte(uint8_t chan, uint8_t data);
bool vcpTransmitEmpty(uint8_t chan);

void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);
void USB_LP_CAN1_RX0_IRQHandler(void);
/* External variables --------------------------------------------------------*/

#endif  /*__HW_CONFIG_H*/
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
