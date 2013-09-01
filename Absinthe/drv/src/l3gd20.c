
#include "main.h"

// L3G4200D, Standard address 0x6A
#define L3GD20_ADDRESS         	0x6A
#define L3GD20_ID              	0xD4

// Registers
#define L3GD20_WHO_AM_I        	0x0F
#define L3GD20_CTRL_REG1       	0x20
#define L3GD20_CTRL_REG2       	0x21
#define L3GD20_CTRL_REG3       	0x22
#define L3GD20_CTRL_REG4       	0x23
#define L3GD20_CTRL_REG5       	0x24
#define L3GD20_REFERENCE       	0x25
#define L3GD20_STATUS_REG      	0x27
#define L3GD20_GYRO_OUT        	0x28
#define L3GD20_FIFO_CTRL_REG   	0x2E
#define L3GD20_FIFO_SRC_REG    	0x2F


// CTRL_REG1 bits
#define L3GD20_POWER_ON        	0x0F
#define L3GD20_ODR_95HZ        	0x00
#define L3GD20_ODR_190HZ       	0x40
#define L3GD20_ODR_380HZ       	0x80
#define L3GD20_ODR_760HZ       	0xC0

// CTRL_REG3 bits
#define L3GD20_I2_DRDY			0x08

// CTRL_REG4 bits
#define L3GD20_FS_SEL_2000DPS	0xF0

// CTRL_REG5 bits
#define L3GD20_FIFO_EN			0x40

// FIFO_CTRL_REG
#define L3GD20_BYPASS_MODE		0x00
#define L3GD20_FIFO_MODE		0x20
#define L3GD20_STREAM_MODE		0x40
#define L3GD20_STREAM2FIFO_MODE	0x60
#define L3GD20_BYPASS2STREAM_MODE	0x80


#if USE_SENSOR_IRQ
uint32_t l3gd20_irq_count = 0;

// GIRO_INT2 - PC3 - DRDY/INT2

/**
 * @brief  This function handles GYRO_INT2 External lines interrupt request.
 * @param  None
 * @retval None
 */
void GYRO_INT2_IRQHandler(void)
{
	l3gd20_irq_count++;

	/* Clear the GYRO_INT2 EXTI line pending bit */
	EXTI_ClearITPendingBit(GYRO_INT2_EXTI_LINE);
}

static void l3gd20IrqConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIOC clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	/* Configure GYRO_INT2 pin as input floating */
	GPIO_InitStructure.GPIO_Pin = GYRO_INT2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GYRO_INT2_GPIO_PORT, &GPIO_InitStructure);

	/* Enable AFIO clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	/* Connect EXTI Line to PC1 pin */
	GPIO_EXTILineConfig(GYRO_INT2_EXTI_PORT_SRC, GYRO_INT2_EXTI_PIN_SRC);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = GYRO_INT2_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI1 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = GYRO_INT2_IRQCHANNEL;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
#else
#define     l3gd20IrqConfig()	;
#endif


bool l3gd20Config(void)
{
    bool ack;

    ack = i2cWrite(L3GD20_ADDRESS, L3GD20_CTRL_REG4, L3GD20_FS_SEL_2000DPS);
    if (!ack) return false;

    i2cWrite(L3GD20_ADDRESS, L3GD20_CTRL_REG1, L3GD20_POWER_ON | L3GD20_ODR_760HZ);
#if USE_SENSOR_IRQ
    i2cWrite(L3GD20_ADDRESS, L3GD20_CTRL_REG3, L3GD20_I2_DRDY);
#endif
    i2cWrite(L3GD20_ADDRESS, L3GD20_CTRL_REG5, L3GD20_FIFO_EN);

    i2cWrite(L3GD20_ADDRESS, L3GD20_FIFO_CTRL_REG, L3GD20_STREAM_MODE);

    return true;
}

bool l3gd20Detect(void)
{
    uint8_t deviceid;

    i2cRead(L3GD20_ADDRESS, L3GD20_WHO_AM_I, 1, &deviceid);
    if (deviceid != L3GD20_ID)
        return false;

    l3gd20IrqConfig();

    if (!l3gd20Config())
    	return false;

    return true;
}

// Read 3 gyro values into user-provided buffer. No overrun checking is done.
void l3gd20Read(int16_t *gyroData)
{
    uint8_t buf[6];
    i2cRead(L3GD20_ADDRESS, L3GD20_GYRO_OUT | 0x80, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    gyroData[2] = -(int16_t)((buf[4] << 8) | buf[5]);
}

// Return FIFO stored data level
uint8_t l3gd20GetFIFOLevel(void)
{
	uint8_t level;

	if (!i2cRead(L3GD20_ADDRESS, L3GD20_FIFO_SRC_REG, 1, &level))
		return MEMS_ERROR;

	level &= 0x0F;

	return level;
}

// Return gyro temperature
uint8_t l3gd20GetTemp(int16_t * temp)
{
	uint8_t value;

	if (!i2cRead(L3GD20_ADDRESS, L3GD20_FIFO_SRC_REG, 1, &value))
		return MEMS_ERROR;

	*temp = (int16_t) (value);
	return MEMS_SUCCESS;
}
