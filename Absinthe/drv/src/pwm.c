#include "main.h"

#define PULSE_1MS       (1000) // 1ms pulse width

typedef void pwmCallbackPtr(uint8_t port, uint16_t capture);

static pwmHardware_t timerHardware[] = {
    { 0, GPIOB, GPIO_Pin_9,  TIM4, TIM_Channel_4, TIM4_IRQn,    0, },       				// SERVO1 /	GROUP1
    { 0, GPIOB, GPIO_Pin_8,  TIM4, TIM_Channel_3, TIM4_IRQn,    0, },       				// SERVO2 /	GROUP1
    { 0, GPIOB, GPIO_Pin_7,  TIM4, TIM_Channel_2, TIM4_IRQn,    0, },       				// SERVO3 /	GROUP1
    { 0, GPIOB, GPIO_Pin_6,  TIM4, TIM_Channel_1, TIM4_IRQn,    0, },       				// SERVO4 /	GROUP1
    { 0, GPIOB, GPIO_Pin_5,  TIM3, TIM_Channel_2, TIM3_IRQn,    GPIO_PartialRemap_TIM3, },  // SERVO5 /	GROUP2
    { 0, GPIOB, GPIO_Pin_4,  TIM3, TIM_Channel_1, TIM3_IRQn,    GPIO_PartialRemap_TIM3, },  // SERVO6 /	GROUP2
    { 0, GPIOB, GPIO_Pin_3,  TIM2, TIM_Channel_2, TIM2_IRQn,    GPIO_PartialRemap1_TIM2, }, // SERVO5 /	GROUP2
    { 0, GPIOA, GPIO_Pin_15, TIM2, TIM_Channel_1, TIM2_IRQn,    GPIO_PartialRemap1_TIM2, }, // SERVO6 /	GROUP2
    { 4, GPIOC, GPIO_Pin_9,  TIM8, TIM_Channel_4, TIM8_CC_IRQn, 0, },       				// PWM5 /	GROUP4
    { 5, GPIOC, GPIO_Pin_8,  TIM8, TIM_Channel_3, TIM8_CC_IRQn, 0, },       				// PWM6 /	GROUP4
    { 6, GPIOC, GPIO_Pin_7,  TIM8, TIM_Channel_2, TIM8_CC_IRQn, 0, },       				// PWM7 /	GROUP4
    { 7, GPIOC, GPIO_Pin_6,  TIM8, TIM_Channel_1, TIM8_CC_IRQn, 0, },       				// PWM8 /	GROUP4
    { 0, GPIOA, GPIO_Pin_0,  TIM5, TIM_Channel_1, TIM5_IRQn,    0, },       				// PWM1 /	GROUP3
    { 1, GPIOA, GPIO_Pin_1,  TIM5, TIM_Channel_2, TIM5_IRQn,    0, },       				// PWM2 /	GROUP3
    { 2, GPIOA, GPIO_Pin_2,  TIM5, TIM_Channel_3, TIM5_IRQn,    0, },       				// PWM3 /	GROUP3
    { 3, GPIOA, GPIO_Pin_3,  TIM5, TIM_Channel_4, TIM5_IRQn,    0, },      					// PWM4 /	GROUP3
};

typedef struct {
    pwmCallbackPtr 		*callback;
    volatile uint16_t 	*ccr;
    uint16_t 			period;

    // for input only
    uint8_t 			state;
    uint16_t 			rise;
    uint16_t 			fall;
    uint16_t 			capture;
} pwmPortData_t;

static pwmPortData_t pwmPorts[MAX_PORTS];
static uint16_t captures[MAX_INPUTS];
static uint16_t failsafeThreshold = 985;
//extern int16_t failsafeCnt;
static uint8_t pwmoutputs = 8;

static void pwmTimeBase(TIM_TypeDef *tim, uint32_t period)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period 		= period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler 	= (SystemCoreClock / 1000000) - 1; // all timers run at 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode 	= TIM_CounterMode_Up;
    TIM_TimeBaseInit(tim, &TIM_TimeBaseStructure);
}

static void pwmNVICConfig(uint8_t irq)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel 					 = irq;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority 		 = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd 				 = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

static void pwmOCConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t value)
{
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode 		 = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse 		 = value;
    TIM_OCInitStructure.TIM_OCPolarity 	 = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;

    switch (channel) {
        case TIM_Channel_1:
            TIM_OC1Init(tim, &TIM_OCInitStructure);
            TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_2:
            TIM_OC2Init(tim, &TIM_OCInitStructure);
            TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_3:
            TIM_OC3Init(tim, &TIM_OCInitStructure);
            TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
        case TIM_Channel_4:
            TIM_OC4Init(tim, &TIM_OCInitStructure);
            TIM_OC4PreloadConfig(tim, TIM_OCPreload_Enable);
            break;
    }
}

static void pwmICConfig(TIM_TypeDef *tim, uint8_t channel, uint16_t polarity)
{
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel 	= channel;
    TIM_ICInitStructure.TIM_ICPolarity  = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter	= 0x0;

    TIM_ICInit(tim, &TIM_ICInitStructure);
}

static void pwmGPIOConfig(GPIO_TypeDef *gpio, uint32_t pin, uint8_t input, uint32_t remap)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin	  = pin;

	if (input)
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;
	else
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(gpio, &GPIO_InitStructure);

	if (remap) GPIO_PinRemapConfig(remap, ENABLE);
}

static pwmPortData_t *pwmOutConfig(uint8_t port, uint16_t period, uint16_t value)
{
    pwmPortData_t *p = &pwmPorts[port];
    pwmTimeBase(timerHardware[port].tim, period);
    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, 0, timerHardware[port].remap);
    pwmOCConfig(timerHardware[port].tim, timerHardware[port].channel, value);

    // Needed only on TIM1, TIM8
    if (timerHardware[port].tim == TIM8)
        TIM_CtrlPWMOutputs(timerHardware[port].tim, ENABLE);

    TIM_Cmd(timerHardware[port].tim, ENABLE);

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            p->ccr = &timerHardware[port].tim->CCR1;
            break;
        case TIM_Channel_2:
            p->ccr = &timerHardware[port].tim->CCR2;
            break;
        case TIM_Channel_3:
            p->ccr = &timerHardware[port].tim->CCR3;
            break;
        case TIM_Channel_4:
            p->ccr = &timerHardware[port].tim->CCR4;
            break;
    }
    return p;
}

static void pwmInConfig(uint8_t port, pwmCallbackPtr callback)
{
    pwmTimeBase(timerHardware[port].tim, 0xFFFF);

    pwmGPIOConfig(timerHardware[port].gpio, timerHardware[port].pin, 1, timerHardware[port].remap);

    pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Rising);

    TIM_Cmd(timerHardware[port].tim, ENABLE);

    pwmNVICConfig(timerHardware[port].irq);

    // set callback before configuring interrupts
    pwmPorts[port].callback = callback;

    switch (timerHardware[port].channel) {
        case TIM_Channel_1:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC1, ENABLE);
            break;
        case TIM_Channel_2:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC2, ENABLE);
            break;
        case TIM_Channel_3:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC3, ENABLE);
            break;
        case TIM_Channel_4:
            TIM_ITConfig(timerHardware[port].tim, TIM_IT_CC4, ENABLE);
            break;
    }
}

/* Find PWM port number by timer & channel*/
int8_t pwmPortLookup(TIM_TypeDef* TIMx, uint16_t TIM_IT)
{
	uint8_t port;

	// Start from 8
	for (port = 8; port < MAX_PORTS; port++)
	{
		if ((timerHardware[port].tim == TIMx) && (timerHardware[port].channel == TIM_IT)) return port;
	}
	return -1;
}

static void pwmTIMxHandler(TIM_TypeDef *tim)
{
    int8_t port = 0;
    uint16_t capture = 0;

    // Generic CC handler for TIM2,3,4,5,8
    if (TIM_GetITStatus(tim, TIM_IT_CC1) == SET) {
        capture = tim->CCR1;
        port = pwmPortLookup(tim, TIM_Channel_1);
        TIM_ClearITPendingBit(tim, TIM_IT_CC1);
    } else if (TIM_GetITStatus(tim, TIM_IT_CC2) == SET) {
        capture = tim->CCR2;
        port = pwmPortLookup(tim, TIM_Channel_2);
        TIM_ClearITPendingBit(tim, TIM_IT_CC2);
    } else if (TIM_GetITStatus(tim, TIM_IT_CC3) == SET) {
        capture = tim->CCR3;
        port = pwmPortLookup(tim, TIM_Channel_3);
        TIM_ClearITPendingBit(tim, TIM_IT_CC3);
    } else if (TIM_GetITStatus(tim, TIM_IT_CC4) == SET) {
        capture = tim->CCR4;
        port = pwmPortLookup(tim, TIM_Channel_4);
        TIM_ClearITPendingBit(tim, TIM_IT_CC4);
    }

    if (port >= 0)
    	pwmPorts[port].callback(port, capture);
}

void TIM2_IRQHandler(void)
{
    pwmTIMxHandler(TIM2);
}

void TIM3_IRQHandler(void)
{
    pwmTIMxHandler(TIM3);
}

void TIM4_IRQHandler(void)
{
    pwmTIMxHandler(TIM4);
}

void TIM5_IRQHandler(void)
{
    pwmTIMxHandler(TIM5);
}

void TIM8_CC_IRQHandler(void)
{
    pwmTIMxHandler(TIM8);
}

static void ppmCallback(uint8_t port, uint16_t capture)
{
    uint16_t diff;
    static uint16_t now;
    static uint16_t last = 0;
    static uint8_t chan = 0;
    static uint8_t GoodPulses;

    last = now;
    now = capture;
    diff = now - last;

    if (diff > 2700) { // Per http://www.rcgroups.com/forums/showpost.php?p=21996147&postcount=3960 "So, if you use 2.5ms or higher as being the reset for the PPM stream start, you will be fine. I use 2.7ms just to be safe."
        chan = 0;
    }
    else
    {
        if (diff > 750 && diff < 2250 && chan < 8)	// 750 to 2250 ms is our 'valid' channel range
        {
            captures[chan] = diff;
            if (chan < 4 && diff > failsafeThreshold)
                GoodPulses |= (1 << chan);      	// if signal is valid - mark channel as OK
            if (GoodPulses == 0x0F)					// If first four chanells have good pulses, clear FailSafe counter
            {
                GoodPulses = 0;
                if (failsafeCnt > 20)
                    failsafeCnt -= 20;
                else
                    failsafeCnt = 0;
            }
        }
        chan++;
        failsafeCnt = 0;
    }
}

static void pwmCallback(uint8_t port, uint16_t capture)
{
    if (pwmPorts[port].state == 0)
    {
        pwmPorts[port].rise = capture;
        pwmPorts[port].state = 1;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Falling);
    }
    else
    {
        // compute capture
        captures[timerHardware[port].pwmchan] = capture - pwmPorts[port].rise;

        // switch state
        pwmPorts[port].state = 0;
        pwmICConfig(timerHardware[port].tim, timerHardware[port].channel, TIM_ICPolarity_Rising);

        // reset failsafe
        failsafeCnt = 0;
    }
}

void pwmInit(uint8_t mode,
		uint16_t pwm_group1_rate,
		uint16_t pwm_group2_rate,
		uint16_t pwm_group3_rate,
		uint16_t pwm_group4_rate)
{
    uint8_t port;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 |
			RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    switch (mode)
    {
    case RC_PWM:	// 8 PWM inputs, 8 PWM outputs in group1 & group2
        pwmoutputs = 8;

        for (port = 8; port < 16; port++)
            pwmInConfig(port, pwmCallback);

        for (port = 0; port < 4; port++)
        	pwmOutConfig(port, 1000000 / pwm_group1_rate, PULSE_1MS);

        for (port = 4; port < 8; port++)
        	pwmOutConfig(port, 1000000 / pwm_group2_rate, PULSE_1MS);
        break;

    case RC_PPM:	// 1 PPM input, 12 PWM outputs in group1, group2 & gpoup3
        pwmoutputs = 12;

        pwmInConfig(PWM1, ppmCallback);

        for (port = 0; port < 4; port++)
            pwmOutConfig(port, 1000000 / pwm_group1_rate, PULSE_1MS);

        for (port = 4; port < 8; port++)
            pwmOutConfig(port, 1000000 / pwm_group2_rate, PULSE_1MS);

        for (port = 8; port < 12; port++)
            pwmOutConfig(port, 1000000 / pwm_group3_rate, PULSE_1MS);
        break;

    case RC_SBUS:
    case RC_SPEKTRUM:
        pwmoutputs = 16;

        for (port = 0; port < 4; port++)
            pwmOutConfig(port, 1000000 / pwm_group1_rate, PULSE_1MS);

        for (port = 4; port < 8; port++)
            pwmOutConfig(port, 1000000 / pwm_group2_rate, PULSE_1MS);

        for (port = 8; port < 12; port++)
            pwmOutConfig(port, 1000000 / pwm_group3_rate, PULSE_1MS);

        for (port = 12; port < 16; port++)
            pwmOutConfig(port, 1000000 / pwm_group4_rate, PULSE_1MS);
    	break;
    }
}

void pwmWrite(uint8_t index, uint16_t value)
{
    if (index < pwmoutputs)
        *pwmPorts[index].ccr = value;
}

uint16_t pwmRead(uint8_t channel)
{
    return captures[channel];
}
