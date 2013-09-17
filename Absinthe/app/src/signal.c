
#include "main.h"

#define	PATTERN_MAX_LENGHT	32

static xTimerHandle LedTimer[3];
static xTimerHandle BuzzerTimer;
static char 		patternChar[PATTERN_MAX_LENGHT];
static uint8_t 		patternPos = PATTERN_MAX_LENGHT;		// Mean stop playing
static uint8_t 		patternLen = 0;
static uint16_t 	Frequency = 3000;
static uint8_t 		Duration = 0;
static uint8_t 		lastDuration = 1;
static bool			playing = false;

gpio_config_t gpio_cfg[] = {
    { LED0_GPIO, LED0_PIN, GPIO_Mode_Out_PP }, // LED STS
    { LED1_GPIO, LED1_PIN, GPIO_Mode_Out_PP }, // LED ERR
    { LED2_GPIO, LED2_PIN, GPIO_Mode_Out_PP }, // LED NAV
};

uint8_t gpio_count = sizeof(gpio_cfg) / sizeof(gpio_cfg[0]);

void ledInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    /* Configure gpio */
    for (i = 0; i < gpio_count; i++) {
        GPIO_InitStructure.GPIO_Pin   = gpio_cfg[i].pin;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
        GPIO_InitStructure.GPIO_Mode  = gpio_cfg[i].mode;
        GPIO_Init(gpio_cfg[i].gpio, &GPIO_InitStructure);

        /* Switch off all LEDs */
        signalLED(i, LEDMODE_OFF);
    }

}

// Set up buzzer peripheral
void buzzerInit(void)
{
	RCC_APB2PeriphClockCmd(BUZZER_RCC_PORT, ENABLE);

	/* Configure the pin */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = BUZZER_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;		// Buzer off
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStructure);

	/* Configure the channels 1 Timer1 to be in output compare mode */
	TIM_OCInitTypeDef TIM_OCInitStruct;
	TIM_OCInitStruct.TIM_OCMode 		= TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState 	= TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OutputNState 	= TIM_OutputNState_Disable;
	TIM_OCInitStruct.TIM_Pulse 			= 200;
	TIM_OCInitStruct.TIM_OCPolarity 	= TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCNPolarity 	= TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OCIdleState 	= TIM_OCIdleState_Reset;
	TIM_OCInitStruct.TIM_OCNIdleState 	= TIM_OCNIdleState_Reset;
	TIM_OC1Init(BUZZER_TIMER, &TIM_OCInitStruct);

	TIM_OC1PreloadConfig(BUZZER_TIMER, TIM_OCPreload_Enable);

#define FREQ		(3000)						// Hz
#define DURATION	((1000000 / FREQ) / 2)

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_ClockDivision 	= TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode 		= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Prescaler 		= (SystemCoreClock / 1000000) - 1;
	TIM_TimeBaseStructure.TIM_Period 			= ((1000000 / FREQ) - 1);
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(BUZZER_TIMER, &TIM_TimeBaseStructure);

	TIM_SetCompare1(BUZZER_TIMER, DURATION);
	TIM_ARRPreloadConfig(BUZZER_TIMER, ENABLE);
	TIM_CtrlPWMOutputs(BUZZER_TIMER, ENABLE);
	TIM_Cmd(BUZZER_TIMER, ENABLE);
}

/* Switch on buzzer */
void buzzerOn(void)
{
	/* Configure the pin to output */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = BUZZER_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStructure);
}

/* Switch off buzzer */
void buzzerOff(void)
{
	/* Configure the pin to input */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin   = BUZZER_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStructure);
}

/*
 *	Set buzzer frequency.
 *	Best value 3000 (Hz)
 *	Acceptable values 400-4000 (not strictly).
 */
void buzzerFreq(uint16_t freq)
{
	BUZZER_TIMER->ARR = ((1000000 / freq) - 1);
	TIM_SetCompare1(BUZZER_TIMER, (1000000 / freq) / 2);
}

#if 0
void buzzer(uint8_t warn_vbat)
{
    static uint8_t beeperOnBox;
    static uint8_t warn_noGPSfix = 0;
    static uint8_t warn_failsafe = 0;
    static uint8_t warn_runtime = 0;

    //=====================  BeeperOn via rcOptions =====================
    if (rcOptions[BOXBEEPERON]) {       // unconditional beeper on via AUXn switch 
        beeperOnBox = 1;
    } else {
        beeperOnBox = 0;
    }
    //===================== Beeps for failsafe =====================
    if (cfg.failsafe) {
        if (failsafeCnt > (5 * cfg.failsafe_delay) && flag(FLAG_ARMED)) {
            warn_failsafe = 1;      //set failsafe warning level to 1 while landing
            if (failsafeCnt > 5 * (cfg.failsafe_delay + cfg.failsafe_off_delay))
                warn_failsafe = 2;  //start "find me" signal after landing   
        }
        if (failsafeCnt > (5 * cfg.failsafe_delay) && !flag(FLAG_ARMED))
            warn_failsafe = 2;      // tx turned off while motors are off: start "find me" signal
        if (failsafeCnt == 0)
            warn_failsafe = 0;      // turn off alarm if TX is okay
    }

    //===================== GPS fix notification handling =====================
    if (sensors(SENSOR_GPS)) {
        if ((rcOptions[BOXGPSHOME] || rcOptions[BOXGPSHOLD]) && !flag(FLAG_GPS_FIX)) {     // if no fix and gps funtion is activated: do warning beeps
            warn_noGPSfix = 1;
        } else {
            warn_noGPSfix = 0;
        }
    }

    //===================== Priority driven Handling =====================
    // beepcode(length1,length2,length3,pause)
    // D: Double, L: Long, M: Middle, S: Short, N: None
    if (warn_failsafe == 2)
        beep_code('L','N','N','D');                 // failsafe "find me" signal
    else if (warn_failsafe == 1)
        beep_code('S','M','L','M');                 // failsafe landing active
    else if (warn_noGPSfix == 1)
        beep_code('S','S','N','M');
    else if (beeperOnBox == 1)
        beep_code('S','S','S','M');                 // beeperon
    else if (warn_vbat == 4)
        beep_code('S','M','M','D');
    else if (warn_vbat == 2)
        beep_code('S','S','M','D');
    else if (warn_vbat == 1)
        beep_code('S','M','N','D');
    else if (warn_runtime == 1 && flag(FLAG_ARMED))
        beep_code('S','S','M','N');                 // Runtime warning
    else if (toggleBeep > 0)
        beep(50);                                   // fast confirmation beep
    else { 
        buzzerIsOn = 0;
        //BEEP_OFF;
    }
}

#endif

/*
 	 Play the notes patern.

 	 Use standart alfabetic notes (A-G) and simbol '#' and 'b'
 	 Digits 1,2,4,8 change notes duration.
 	 Simbols 'X', 'L', 'M' generate frequency 3000, 2700 and 2500 Hz
 	 Any other simbol plays as pause.
 	 By defualt used last duration.

 	 Example: beep_play("1A B 2C#");
 */
void buzzerPlay(char * pattern)
{
    strncpy(patternChar, pattern, PATTERN_MAX_LENGHT - 1);

    patternLen = strlen(patternChar);

    if (patternLen == 0)	// Nothing to play
    	return;

    if (patternLen >= PATTERN_MAX_LENGHT)	// Cut long pattern
    	patternChar[PATTERN_MAX_LENGHT - 1] = 0;

    // Start playing
    patternPos = 0;
    Duration = 0;
    playing = true;
}

void beep_handler(void)
{
	uint8_t getNext;

	if (!playing)
		return;

	if (Duration > 0)
	{
		Duration--;
		return;
	}

	do
	{
		if (patternPos >= patternLen)
		{
			// Stop playing
			buzzerOff();
			playing = false;
			return;
		}

		Duration = lastDuration;
		getNext = false;

		switch(patternChar[patternPos++]) {
			case '1': Duration = 1; getNext = true; break;
			case '2': Duration = 2; getNext = true; break;
			case '4': Duration = 4; getNext = true; break;
			case '8': Duration = 8; getNext = true; break;
			case 'A': Frequency = 1760; 			break;
			case 'B': Frequency = 1976; 			break;
			case 'C': Frequency = 1046; 			break;
			case 'D': Frequency = 1175; 			break;
			case 'E': Frequency = 1319; 			break;
			case 'F': Frequency = 1397; 			break;
			case 'G': Frequency = 1568; 			break;
			case 'X': Frequency = 3000; 			break;
			case 'M': Frequency = 2850; 			break;
			case 'L': Frequency = 2700; 			break;
			default:  Frequency = 0;				break;
		}

		lastDuration = Duration;

		if (patternPos < patternLen)
		{
			if (patternChar[patternPos] == 'b')
			{
				Frequency = ((float) Frequency) / 1.05946f;
				patternPos++;
			}
			else if (patternChar[patternPos] == '#')
			{
				Frequency = ((float) Frequency) * 1.05946f;
				patternPos++;
			}
		}
	} while (getNext);

	if (Frequency)
	{
		// Play à note
		buzzerFreq(Frequency);
		buzzerOn();
	}

	else
		buzzerOff();			// Play à pause
}

void signalLED(LEDNUM_t led_num, LEDMODE_t led_mode)
{
	switch (led_mode)
	{

	case LEDMODE_OFF:
		digitalLo(gpio_cfg[led_num].gpio, gpio_cfg[led_num].pin);
		break;

	case LEDMODE_ON:
		digitalHi(gpio_cfg[led_num].gpio, gpio_cfg[led_num].pin);
		break;

	case LEDMODE_TOGGLE:
		digitalToggle(gpio_cfg[led_num].gpio, gpio_cfg[led_num].pin);
		break;

	case LEDMODE_BLINK1:
		digitalHi(gpio_cfg[led_num].gpio, gpio_cfg[led_num].pin);
		xTimerChangePeriod(LedTimer[led_num], 500, 10);
		break;

	case LEDMODE_BLINK2:

		break;
	}
}

void signalBUZZ(BUZZMODE_t buzz_mode)
{
	switch (buzz_mode)
	{
	case BUZZMODE_OFF:
		buzzerOff();
		break;

	case BUZZMODE_ON:
		buzzerOn();
		break;

	case BUZZMODE_1:
		buzzerFreq(3000);
		buzzerOn();
		xTimerChangePeriod(BuzzerTimer, 200, 10);
		break;

	case BUZZMODE_2:
		buzzerFreq(2500);
		buzzerOn();
		xTimerChangePeriod(BuzzerTimer, 500, 10);
		break;
	}
}

void LedTimerCallback(xTimerHandle pxTimer)
{
	signalLED((uint32_t) pvTimerGetTimerID(pxTimer), LEDMODE_OFF);
}

void BuzzerTimerCallback(xTimerHandle pxTimer)
{
	buzzerOff();
}

portTASK_FUNCTION_PROTO(signalTask, pvParameters)
{
	portTickType xLastWakeTime;

	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();

	/* Create timers for LED & buzzer */
	LedTimer[LED_STS] = xTimerCreate((signed char *) "TimLedSTS", 10, pdFALSE, (void *) LED_STS, LedTimerCallback);
	LedTimer[LED_ERR] = xTimerCreate((signed char *) "TimLedERR", 10, pdFALSE, (void *) LED_ERR, LedTimerCallback);
	LedTimer[LED_NAV] = xTimerCreate((signed char *) "TimLedNAV", 10, pdFALSE, (void *) LED_NAV, LedTimerCallback);
	BuzzerTimer = xTimerCreate((signed char *) "TimBuzzer", 10, pdFALSE, (void *) 0, BuzzerTimerCallback);

	buzzerInit();

	ledInit();

    while (1)
    {
    	beep_handler();

		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, 100);	// Task cycle time 100 ms
    }
}
