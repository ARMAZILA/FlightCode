#include "main.h"

// current uptime for 1kHz systick timer. will rollover after 49 days. hopefully we won't care.
volatile uint32_t sysTickUptime = 0;
volatile uint32_t ulHighFrequencyTimerTicks;

// SysTick
// Moved to FreeRTOSHooks.c
#if 0
void SysTick_Handler(void)
{
	static uint16_t second_counter = 0;
	if (second_counter++ == 1000)
	{
		second_counter = 0;
		rtcTickdHandler();
	}
    sysTickUptime++;
}
#endif

#if (configGENERATE_RUN_TIME_STATS == 1)
// Generic update handler for TIM7
void TIM7_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	ulHighFrequencyTimerTicks++;
}
#endif

/*
 * 	TIM6 - used for counting microseconds
 * 	TIM7 - user for counting ulHighFrequencyTimerTicks++;
 */
void sysTimerConfig(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period 	= 1024;
	TIM_TimeBaseStructure.TIM_Prescaler = 69;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM6, ENABLE);

#if (configGENERATE_RUN_TIME_STATS == 1)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period 	= 100;								// timer update period 100 microsecond
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;	// timers run at 1MHz
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* TIM enable counter */
	TIM_Cmd(TIM7, ENABLE);

	/* TIM IT enable */
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* NVIC Configuration */
	NVIC_InitStructure.NVIC_IRQChannel 					 = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority 		 = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd 				 = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
#endif
}

// Return system uptime in microseconds (rollover in 70minutes)
uint32_t micros(void)
{
#if 1
	register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (72000 - cycle_cnt) / 72;
#else
    uint32_t ms = sysTickUptime;


    return 0;
#endif
}

// Return system uptime in milliseconds (rollover in 49 days)
uint32_t millis(void)
{
    return sysTickUptime;
}


static void _putc(void *p, char c)
{
	vcp1SendByte(c);
}

void systemInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable PWR and BKP clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

    /* Enable clocks for on stuff we use */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |
    		RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    /* Make all GPIO in by default to save power and reduce noise */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Turn off JTAG port 'cause we're using the PA15 pin */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* Configure TIM6 & TIM7 */
    sysTimerConfig();

    /* redirect printf to USB VCP */
    init_printf(NULL, _putc);
}

#if 1
void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us);
}
#else
void delayMicroseconds(uint32_t us)
{
    uint32_t elapsed = 0;
    uint32_t lastCount = SysTick->VAL;

    for (;;) {
        register uint32_t current_count = SysTick->VAL;
        uint32_t elapsed_us;

        // measure the time elapsed since the last time we checked
        elapsed += current_count - lastCount;
        lastCount = current_count;

        // convert to microseconds
        elapsed_us = elapsed / usTicks;
        if (elapsed_us >= us)
            break;

        // reduce the delay by the elapsed time
        us -= elapsed_us;

        // keep fractional microseconds for the next iteration
        elapsed %= usTicks;
    }
}
#endif

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

void failureMode(uint8_t mode)
{
	signalLED(LED_ERR, LEDMODE_ON);
	signalLED(LED_STS, LEDMODE_OFF);

	while (1)
    {
		signalLED(LED_ERR, LEDMODE_TOGGLE);
		signalLED(LED_STS, LEDMODE_TOGGLE);
        delay(475 * mode - 2);
        buzzerOn();
        delay(25);
        buzzerOff();
    }
}

void systemReset(bool toBootloader)
{
	if (toBootloader)
	{
		/* Enable access to Backup domain */
		PWR_BackupAccessCmd(ENABLE);

		/* Write magic number to the user backup register telling the bootloader to start DFU mode */
		BKP_WriteBackupRegister(BKP_DR1, MAGIC_NUMBER);
	}

	/* Restart CPU */
	NVIC_SystemReset();
}
