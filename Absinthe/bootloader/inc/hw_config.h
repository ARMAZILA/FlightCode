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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported constants --------------------------------------------------------*/
#define MAGIC_NUMBER		(0xCA6B)	//	Magic word to access bootloader from main programm

#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#define digitalToggle(p, i) { p->ODR ^= i; }


#define USB_DISCONNECT_PORT                 GPIOA
#define USB_DISCONNECT_PIN                  GPIO_Pin_12
#define RCC_APB2Periph_GPIO_DISCONNECT      RCC_APB2Periph_GPIOA

// Hardware definitions and GPIO
#define LED0_GPIO   GPIOC
#define LED0_PIN    GPIO_Pin_14
#define LED1_GPIO   GPIOC
#define LED1_PIN    GPIO_Pin_13
#define LED2_GPIO   GPIOC
#define LED2_PIN    GPIO_Pin_12

/* Exported macro ------------------------------------------------------------*/
//LED STS
#define LED_STS_TOGGLE              digitalToggle(LED0_GPIO, LED0_PIN);
#define LED_STS_OFF                 digitalHi(LED0_GPIO, LED0_PIN);
#define LED_STS_ON                  digitalLo(LED0_GPIO, LED0_PIN);

//LED ERR
#define LED_ERR_TOGGLE              digitalToggle(LED1_GPIO, LED1_PIN);
#define LED_ERR_OFF                 digitalHi(LED1_GPIO, LED1_PIN);
#define LED_ERR_ON                  digitalLo(LED1_GPIO, LED1_PIN);

//LED ERR
#define LED2_TOGGLE              digitalToggle(LED2_GPIO, LED2_PIN);
#define LED2_OFF                 digitalHi(LED2_GPIO, LED2_PIN);
#define LED2_ON                  digitalLo(LED2_GPIO, LED2_PIN);

/* Flash memory address from where user application will be loaded */
#define ApplicationAddress 0x08004000

/* Exported functions ------------------------------------------------------- */
void Set_System(void);
void Set_USBClock(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Cable_Config (FunctionalState NewState);
void USB_Interrupts_Config(void);
uint8_t  DFU_Key_Present(void);
void Reset_Device(void);
void Get_SerialNum(void);
void bl_delay_ms(__IO uint32_t nCount);
void bl_delay(__IO uint32_t nCount);	// 7200 for 1 millisecond
uint8_t CheckBackupReg(void);


#endif  /*__HW_CONFIG_H*/
