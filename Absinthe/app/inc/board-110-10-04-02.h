#ifndef BOARD_110_10_04_02_H
#define BOARD_110_10_04_02_H

/*
	Armazila board 110-10-04-02 (10dM3UOP88) hardware definitions
*/

/*
	ADC signals
-----------------------------------------------------------------------
 #		Describtion				Sch			Rate	Pin		ADC Channel
-----------------------------------------------------------------------
 ADC(0)	Video battery (Voltage)	OSD_VBAT	1:6		PB1 	ADC12_IN9
 ADC(1)	Power sensor (Voltage) 	ADC_VOLT	1:6		PA5 	ADC12_IN5
 ADC(2)	Power sensor (Current)	ADC_CURR	1:2		PA6 	ADC12_IN6
 ADC(3)	MCU Temp Sensor			-			-		-	 	TempSensor
*/

/*
	PWM Input timers
----------------------------
 #		Timer/Channel	Pin
----------------------------
 PWM1	TIM5_CH1		PA0
 PWM2	TIM5_CH2		PA1
 PWM3	TIM5_CH3		PA2
 PWM4	TIM5_CH4		PA3
 PWM5	TIM8_CH4		PC9
 PWM6	TIM8_CH3		PC8
 PWM7	TIM8_CH2		PC7
 PWM8	TIM8_CH1		PC6

	SERVO Output timers
----------------------------
 #		Timer/Channel	Pin
----------------------------
 SERVO1	TIM4_CH4		PB9
 SERVO2	TIM4_CH3		PB8
 SERVO3	TIM4_CH2		PB7
 SERVO4	TIM4_CH1		PB6
 SERVO5	TIM3_CH2		PB5
 SERVO6	TIM3_CH1		PB4
 SERVO7	TIM2_CH2		PB3
 SERVO8	TIM2_CH1		PA15

*/

/*
	OSD
--------------------------
Signal		Func		Pin
----------------------------
OSD_HSYNC	EXTI0		PC0
OSD_VSYNC	EXTI1		PC1
OSD_BW		SPI1_MOSI	PA7
OSD_MASK	SPI2_MOSI	PB15
OSD_AUDIO	DAC1		PA4
 */
#define OSD_HSYNC_IRQHandler 	EXTI0_IRQHandler
#define	OSD_HSYNC_EXTI_Line		EXTI_Line0
#define	OSD_HSYNC_EXTI_PORT		GPIOC
#define	OSD_HSYNC_EXTI_PIN		GPIO_Pin_0
#define	OSD_HSYNC_RCC_PORT		RCC_APB2Periph_GPIOC
#define	OSD_HSYNC_EXTI_PORT_SRC	GPIO_PortSourceGPIOC
#define	OSD_HSYNC_EXTI_PIN_SRC	GPIO_PinSource0
#define	OSD_HSYNC_IRQCHANNEL	EXTI0_IRQn

#define OSD_VSYNC_IRQHandler 	EXTI1_IRQHandler
#define	OSD_VSYNC_EXTI_LINE		EXTI_Line1
#define	OSD_VSYNC_EXTI_PORT		GPIOC
#define	OSD_VSYNC_EXTI_PIN		GPIO_Pin_1
#define	OSD_VSYNC_RCC_PORT		RCC_APB2Periph_GPIOC
#define	OSD_VSYNC_EXTI_PORT_SRC	GPIO_PortSourceGPIOC
#define	OSD_VSYNC_EXTI_PIN_SRC	GPIO_PinSource1
#define	OSD_VSYNC_IRQCHANNEL	EXTI1_IRQn

/* LEDs */
#define LED0_GPIO   GPIOC
#define LED0_PIN    GPIO_Pin_14
#define LED1_GPIO   GPIOC
#define LED1_PIN    GPIO_Pin_13
#define LED2_GPIO   GPIOB
#define LED2_PIN    GPIO_Pin_14

/* Buzzer - PA8 - TIM1_CH1 */
#define	BUZZER_GPIO_PORT		GPIOA
#define	BUZZER_GPIO_PIN			GPIO_Pin_8
#define	BUZZER_RCC_PORT			RCC_APB2Periph_TIM1
#define	BUZZER_TIMER			TIM1

/*
 *	UARTs
 *
 *	UART1 - Bloetooth, XBee, Radio modem, ... (MSP / MAVLink protocol)
 *	UART3 - GPS
 *	UART5 - FrSky telemetry, S.Bus
 *
 * -----------------------------
 * UART#		FUNC		PIN
 * -----------------------------
 * UART1_TX 	UART1_TX	PA9
 * UART1_RX 	UART1_RX	PA10
 * UART3_TX		GPS_TX		PC10
 * UART3_RX		GPS_RX		PC11
 * UART5_TX 	TELE_TX		PC12
 * UART5_RX 	TELE_RX		PD2
 */

#define UART1_TX_GPIO_PORT	GPIOA
#define UART1_TX_GPIO_PIN	GPIO_Pin_9
#define UART1_RX_GPIO_PORT	GPIOA
#define UART1_RX_GPIO_PIN	GPIO_Pin_10


/*
 * 	HC-SR04 Sonar connecting to AUX port
 *
 * 	AUX_TRIG - PB12
 * 	AUX_ECHO - PB13 - EXTI13
 */
#define SONAR_TRIG_GPIO_PORT		GPIOB
#define SONAR_TRIG_GPIO_PIN			GPIO_Pin_12
#define SONAR_ECHO_GPIO_PORT		GPIOB
#define SONAR_ECHO_GPIO_PIN			GPIO_Pin_13
#define SONAR_ECHO_EXTI_PORT_SRC	GPIO_PortSourceGPIOB
#define SONAR_ECHO_EXTI_PIN_SRC 	GPIO_PinSource13
#define SONAR_ECHO_EXTI_LINE		EXTI_Line13
#define SONAR_ECHO_IRQCHANNEL		EXTI15_10_IRQn
#define SONAR_ECHO_IRQHandler 		EXTI15_10_IRQHandler


#define USE_SENSOR_IRQ				0	// We don't use sensors interrupts

/*
 * 	Gyro L3GD20
 *
 *  GIRO_INT2 - PC3 - DRDY/INT2
 */
#define GYRO_INT2_GPIO_PORT			GPIOC
#define GYRO_INT2_GPIO_PIN			GPIO_Pin_3
#define GYRO_INT2_EXTI_PORT_SRC		GPIO_PortSourceGPIOC
#define GYRO_INT2_EXTI_PIN_SRC 		GPIO_PinSource3
#define GYRO_INT2_EXTI_LINE			EXTI_Line3
#define GYRO_INT2_IRQCHANNEL		EXTI3_IRQn
#define GYRO_INT2_IRQHandler 		EXTI3_IRQHandler

/*
 * 	Baro sensor LPS331AP
 *
 *  BARO_INT1 - PC2
 */
#define BARO_INT1_GPIO_PORT			GPIOC
#define BARO_INT1_GPIO_PIN			GPIO_Pin_2
#define BARO_INT1_EXTI_PORT_SRC		GPIO_PortSourceGPIOC
#define BARO_INT1_EXTI_PIN_SRC 		GPIO_PinSource2
#define BARO_INT1_EXTI_LINE			EXTI_Line2
#define BARO_INT1_IRQCHANNEL		EXTI2_IRQn
#define BARO_INT1_IRQHandler 		EXTI2_IRQHandler

/*
 * 	Accel/Compass LSM303DLHC
 *
 *  ACCEL_INT1	- PC4
 *  MAG_DRDY	- PC5
 */
#define ACCEL_INT1_GPIO_PORT		GPIOC
#define ACCEL_INT1_GPIO_PIN			GPIO_Pin_4
#define ACCEL_INT1_EXTI_PORT_SRC	GPIO_PortSourceGPIOC
#define ACCEL_INT1_EXTI_PIN_SRC 	GPIO_PinSource4
#define ACCEL_INT1_EXTI_LINE		EXTI_Line4
#define ACCEL_INT1_IRQCHANNEL		EXTI4_IRQn
#define ACCEL_INT1_IRQHandler 		EXTI4_IRQHandler
#define MAG_DRDY_GPIO_PORT			GPIOC
#define MAG_DRDY_GPIO_PIN			GPIO_Pin_5
#define MAG_DRDY_EXTI_PORT_SRC		GPIO_PortSourceGPIOC
#define MAG_DRDY_EXTI_PIN_SRC 		GPIO_PinSource5
#define MAG_DRDY_EXTI_LINE			EXTI_Line5
#define MAG_DRDY_IRQCHANNEL			EXTI9_5_IRQn
#define MAG_DRDY_IRQHandler 		EXTI9_5_IRQHandler

#endif
