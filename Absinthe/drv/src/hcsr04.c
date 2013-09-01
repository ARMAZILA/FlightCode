#include "main.h"

/* HC-SR04 consists of ultrasonic transmitter, receiver, and control circuits.
 * When trigged it sends out a series of 40KHz ultrasonic pulses and receives
 * echo froman object. The distance between the unit and the object is calculated
 * by measuring the traveling time of sound and output it as the width of a TTL pulse.
 *
 * *** Warning: HC-SR04 operates at +5V ***
 *
 */

static volatile int16_t sonar_distance = -1;
static volatile uint8_t sonar_updated = 0;
int32_t pulse_duration;

void SONAR_ECHO_IRQHandler(void)
{
    static uint32_t timing_start;

    if (GPIO_ReadInputDataBit(SONAR_ECHO_GPIO_PORT, SONAR_ECHO_GPIO_PIN) != 0)
        timing_start = micros();
    else 
    {
        uint32_t timing_stop = micros();
        if (timing_stop > timing_start)
        {
            pulse_duration = timing_stop - timing_start;
            sonar_updated = 1;
        }
    }

    EXTI_ClearITPendingBit(SONAR_ECHO_EXTI_LINE);
}

void hcsr04_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    //enable AFIO for EXTI support - already done is system.c
    //RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph, ENABLE); 
    
    // tp - trigger pin 
    GPIO_InitStructure.GPIO_Pin   = SONAR_TRIG_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(SONAR_TRIG_GPIO_PORT, &GPIO_InitStructure);

    // ep - echo pin
    GPIO_InitStructure.GPIO_Pin  = SONAR_ECHO_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(SONAR_ECHO_GPIO_PORT, &GPIO_InitStructure);

    // setup external interrupt on echo pin
    GPIO_EXTILineConfig(SONAR_ECHO_EXTI_PORT_SRC, SONAR_ECHO_EXTI_PIN_SRC);

    EXTI_ClearITPendingBit(SONAR_ECHO_EXTI_LINE);
       
    EXTI_InitStructure.EXTI_Line 	= SONAR_ECHO_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode 	= EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_EnableIRQ(SONAR_ECHO_IRQCHANNEL);
}

/*
 * distance calculation is done asynchronously, using interrupt
 * the repeat interval of trig signal should be greater than 60ms
 * Input  : temp - temperature in degrees Celsius
 * Return : distance in cantimeters
 *        : -1 if no sonar echoes
 *
 *	 https://en.wikipedia.org/wiki/Speed_of_sound
 *
 *	 The approximate speed of sound in dry (0% humidity) air, in meters per second,
 *	 at temperatures near 0 °C, can be calculated from:
 *
 *	 V = 331.3 + 0.606 * T
 *
 *	 where T is the temperature in degrees Celsius (°C).
 */
void hcsr04_get_distance(int16_t temp, int16_t* distance)
{
	if (sonar_updated)
	{
		float cm_per_micros = (331.3f + 0.606f * temp) / 20000.0f;
	    sonar_distance = pulse_duration * cm_per_micros;
	    sonar_updated = 1;

		*distance = sonar_distance;
	}
	else
		*distance = -1;

    //  The width of trig signal must be greater than 10us
    GPIO_SetBits(SONAR_TRIG_GPIO_PORT, SONAR_TRIG_GPIO_PIN);
    delayMicroseconds(11);
    GPIO_ResetBits(SONAR_TRIG_GPIO_PORT, SONAR_TRIG_GPIO_PIN);

    // Clear sonar update flag
    sonar_updated = 0;
}
