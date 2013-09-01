/**
 ******************************************************************************
 * @file    main.c
 * @author  avgorbi
 * @version V1.0.0
 * @date    05-May-2013
 * @brief   Device Firmware Upgrade(DFU) main file
 ******************************************************************************
 *
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "hw_config.h" 
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_pwr.h"
#include "dfu_mal.h"

/* Private typedef -----------------------------------------------------------*/
typedef void (*pFunction)(void);
typedef struct gpio_config_t
{
	GPIO_TypeDef *gpio;
	uint16_t pin;
	GPIOMode_TypeDef mode;
} gpio_config_t;

/* Private variables ---------------------------------------------------------*/
uint8_t DeviceState;
uint8_t DeviceStatus[6];
pFunction Jump_To_Application;
uint32_t JumpAddress;

/* Private functions ---------------------------------------------------------*/
void HW_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	uint32_t i;

	gpio_config_t gpio_cfg[] =
	{
		{ LED0_GPIO, LED0_PIN,    GPIO_Mode_Out_PP }, // PC12 (LED STS)
		{ LED1_GPIO, LED1_PIN,    GPIO_Mode_Out_PP }, // PC13 (LED ERR)
		{ LED2_GPIO, LED2_PIN,    GPIO_Mode_Out_PP }, // PC14 (LED NAV)
		{ GPIOB,	 GPIO_Pin_10, GPIO_Mode_IPU    }, // PB10 (I2C2_SCL) DFU Key
	};
	uint8_t gpio_count = sizeof(gpio_cfg) / sizeof(gpio_cfg[0]);

	/* Enable PWR and BKP clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	/* Enable access to Backup domain */
	PWR_BackupAccessCmd(ENABLE);

	// Turn on clocks for stuff we use
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

	// Make all GPIO in by default to save power and reduce noise
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	// Configure gpio
	for (i = 0; i < gpio_count; i++)
	{
		GPIO_InitStructure.GPIO_Pin   = gpio_cfg[i].pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
		GPIO_InitStructure.GPIO_Mode  = gpio_cfg[i].mode;
		GPIO_Init(gpio_cfg[i].gpio, &GPIO_InitStructure);
	}

	LED_STS_OFF;
	LED_ERR_OFF;
	LED2_OFF;
}

/*******************************************************************************
 * Function Name  : main.
 * Description    : main routine.
 * Input          : None.
 * Output         : None.
 * Return         : None.
 *******************************************************************************/
int main(void)
{
	HW_Config();

	/* Check if the Bootloader Key is not present */
	if (!DFU_Key_Present() &&
		/* Test if user code is programmed starting from address 0x8004000 */
		((*(__IO uint32_t*) ApplicationAddress) & 0x2FFE0000) == 0x20000000 &&
		/* Check to request to boolloader from main programm */
		!CheckBackupReg()
		)
	{ /* Jump to user application */
			JumpAddress = *(__IO uint32_t*) (ApplicationAddress + 4);
			Jump_To_Application = (pFunction) JumpAddress;

			/* Disable all interrupts */
			__set_PRIMASK(1);

			/* Initialize user application's Stack Pointer */
			__set_MSP(*(__IO uint32_t*) ApplicationAddress);
			Jump_To_Application();
	} /* Otherwise enters DFU mode to allow user to program his application */

	/* Enter DFU mode */
	DeviceState = STATE_dfuERROR;
	DeviceStatus[0] = STATUS_ERRFIRMWARE;
	DeviceStatus[4] = DeviceState;

	Set_System();
	Set_USBClock();
	USB_Init();

	int16_t c = 1;
	int16_t e = 2;

	/* Main loop */
	while (1)
	{
		LED_STS_ON;
		LED_ERR_OFF;

		bl_delay(720 * c);  // ~100 microsecond

		LED_STS_OFF;
		LED_ERR_ON;

		bl_delay(720 * (100 - c));  // ~100 microsecond

		c += e;
		if (c >= 100) { c = 100; e = -2; }
		if (c <= 0)   { c = 0;   e = 2;  }
	}
}

#ifdef USE_FULL_ASSERT
/*******************************************************************************
 * Function Name  : assert_failed
 * Description    : Reports the name of the source file and the source line number
 *                  where the assert_param error has occurred.
 * Input          : - file: pointer to the source file name
 *                  - line: assert_param error line source number
 * Output         : None
 * Return         : None
 *******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}
#endif

