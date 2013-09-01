#pragma once

#define MAX_MOTORS  16
#define MAX_SERVOS  16
#define MAX_INPUTS  8

// This indexes into the read-only hardware definition structure in drv_pwm.c, as well as into pwmPorts[] structure with dynamic data.
enum {
    SERVO1 = 0,
    SERVO2,
    SERVO3,
    SERVO4,
    SERVO5,
    SERVO6,
    SERVO7,
    SERVO8,
    PWM5,
    PWM6,
    PWM7,
    PWM8,
    PWM1,
    PWM2,
    PWM3,
    PWM4,
    MAX_PORTS
};

typedef struct {
	uint8_t			pwmchan;
    GPIO_TypeDef   *gpio;
    uint32_t 		pin;
    TIM_TypeDef    *tim;
    uint8_t 		channel;
    uint8_t 		irq;
    uint32_t 	  	remap;
} pwmHardware_t;

void pwmInit(uint8_t mode,
		uint16_t pwm_group1_rate,
		uint16_t pwm_group2_rate,
		uint16_t pwm_group3_rate,
		uint16_t pwm_group4_rate);
void pwmWrite(uint8_t index, uint16_t value);
uint16_t pwmRead(uint8_t channel);

// void pwmWrite(uint8_t channel, uint16_t value);
