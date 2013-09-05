/*
 *   Armazila Absinth code - for board "110-10-04-02 (10dM3UOP88)"
 *
 *   An Open Source STM32 Based Universal flight control system
 *
 *   Used code and/or ideas of the following projects:
 *
 *   - MultiWii
 *	 - Afrodevices
 *	 - S.O.H. Madgwick
 *	 - Openpilot
 *	 - FreeRTOS
 *	 - STMicroelectronics
 *
 *   To build application firmware use:
 *
 *       make TARGET=10dM3UOP88
 *
 *   To build bootloader use:
 *
 *       make TARGET=BOOTLOADER
 *
 *   Information about the project is available:
 *
 *       http://code.google.com/p/armazila/
 *       https://github.com/ARMAZILA/FlightCode/tree/master/Absinthe
 *       http://armazila.com/flight-controller
 *
 *   Copyright (C) 2013  Armazila
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "main.h"

/* Receivers read functions */

/* USB subsystem functions */
extern void USB_Prepare();
extern void USB_Init();

portTASK_FUNCTION_PROTO(initTask, pvParameters)
{
	/* Configures the priority grouping: pre-emption priority and subpriority.
       2 bits for pre-emption priority and 2 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    systemInit();

	/* Init USB subsystem */
    USB_Prepare();
	USB_Init();

	i2cInit(I2C2);

	adcInit();

	checkFirstTime(false);
	readFlashConfig();

	mixerInit(); // this will set useServo var depending on mixer type
	// when using airplane/wing mixer, servo/motor outputs are remapped

    // Init pwm perephirial
	pwmInit(cfg.rcprotocol,
			cfg.pwm_group1_rate,
			cfg.pwm_group2_rate,
			cfg.pwm_group3_rate,
			cfg.pwm_group4_rate);

	// Configure PWM/CPPM read function
	rcReadRawFunc = rxReadRawRC;		// By default set to PWM

	// Init radio mode
	if (cfg.rcprotocol == RC_SPEKTRUM)
	{
		spektrumInit();
		rcReadRawFunc = spektrumReadRawRC;
	}
	else if (cfg.rcprotocol == RC_SBUS)
	{
		sbusInit();
		rcReadRawFunc = sbusReadRawRC;
	}

	if (cfg.gps_baudrate)
		gpsInit(cfg.gps_baudrate);

	/* Initialize tasks */
	//			Task_func				        Task_name   Stack	   	 Param	 Prio			 Handler
	xTaskCreate(signalTask,  	 (signed char *) "Signal",  256, (void *) NULL,  3, (xTaskHandle *) NULL);
	xTaskCreate(mspTask,     	 (signed char *) "Serial",  512, (void *) NULL,  2, (xTaskHandle *) NULL);
	xTaskCreate(mavlinkTask, 	 (signed char *) "MAVLink", 512, (void *) NULL,  2, (xTaskHandle *) NULL);

	/* Initialize sensors. Do it before starting watch dog timer */
	sensorsInit();

	/* Init Watch-dog. Do not forget call IWDG_ReloadCounter() in time
	   less then 250ms.  We do it in main loop - rcLoopTask */
	wdgInit();

	/* Initialize other tasks */
	//			Task_func				        Task_name   Stack	   	 Param	 Prio			 Handler
	xTaskCreate(sensorTask,  	 (signed char *) "Sensor",  256, (void *) NULL,  4, (xTaskHandle *) NULL);
	xTaskCreate(rcLoopTask,  	 (signed char *) "rcLoop",  256, (void *) NULL,  3, (xTaskHandle *) NULL);
	xTaskCreate(osdTask,     	 (signed char *) "OSD",     256, (void *) NULL,  3, (xTaskHandle *) NULL);
	xTaskCreate(sonarTask,       (signed char *) "Sonar",    48, (void *) NULL,  2, (xTaskHandle *) NULL);
	xTaskCreate(powerSensorTask, (signed char *) "PwrSen",   48, (void *) NULL,  1, (xTaskHandle *) NULL);
	xTaskCreate(navigateTask,  	 (signed char *) "Navi",    256, (void *) NULL,  3, (xTaskHandle *) NULL);

	/* Initialize FrSky telemetry task */
	if (cfg.rcprotocol !=  RC_SBUS)
		xTaskCreate(frskyTask,   (signed char *) "FrSky",  64, (void *) NULL, 1, (xTaskHandle *) NULL);

	/* Terminate initTask */
	vTaskDelete(NULL);
}

int main(void)
{
	/* Check if the system has resumed from IWDG reset */
	if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
	{	/* IWDGRST flag set */
		flagSet(FLAG_WDG_OCCURRED);

		/* Clear reset flags */
		RCC_ClearFlag();
	}
	else
	{	/* IWDGRST flag is not set */
	}

	/* Disable all interrupts */
	__set_PRIMASK(1);

	/* Set the Vector Table base location at 0x4000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x4000);

	/* Enable all interrupts */
	__set_PRIMASK(0);

	//		   Task_func		       Task_name   Stack	    Param  Prio			   Handler
	xTaskCreate(initTask, (signed char *) "Init",  128, (void *) NULL, 2, (xTaskHandle *) NULL);

    /* Start FreeRTOS scheduler */
	vTaskStartScheduler();

	/* This code will never be executed */
	return 0;
}

void HardFault_Handler(void)
{
    // fall out of the sky
	mixerWwriteAllMotors(cfg.mincommand);
    while (1);
}
