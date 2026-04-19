#include "stm32f10x.h"
#include "MCP4725.h"

/* MCP4725 软件I2C引脚定义
   SCL -> PB6
   SDA -> PB7
*/
#define MCP4725_SCL_PORT      GPIOB
#define MCP4725_SCL_PIN       GPIO_Pin_6
#define MCP4725_SDA_PORT      GPIOB
#define MCP4725_SDA_PIN       GPIO_Pin_7

#define MCP4725_W_SCL(x)      GPIO_WriteBit(MCP4725_SCL_PORT, MCP4725_SCL_PIN, (BitAction)(x))
#define MCP4725_W_SDA(x)      GPIO_WriteBit(MCP4725_SDA_PORT, MCP4725_SDA_PIN, (BitAction)(x))

/* MCP4725 默认7位地址 0x60 */
#define MCP4725_ADDR_WRITE    0xC0

static void MCP4725_I2C_Delay(void)
{
	volatile uint8_t i;
	for (i = 0; i < 20; i++);
}

static void MCP4725_I2C_Start(void)
{
	MCP4725_W_SDA(1);
	MCP4725_W_SCL(1);
	MCP4725_I2C_Delay();

	MCP4725_W_SDA(0);
	MCP4725_I2C_Delay();

	MCP4725_W_SCL(0);
	MCP4725_I2C_Delay();
}

static void MCP4725_I2C_Stop(void)
{
	MCP4725_W_SDA(0);
	MCP4725_I2C_Delay();

	MCP4725_W_SCL(1);
	MCP4725_I2C_Delay();

	MCP4725_W_SDA(1);
	MCP4725_I2C_Delay();
}

static void MCP4725_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		if (Byte & 0x80)
		{
			MCP4725_W_SDA(1);
		}
		else
		{
			MCP4725_W_SDA(0);
		}

		MCP4725_I2C_Delay();
		MCP4725_W_SCL(1);
		MCP4725_I2C_Delay();
		MCP4725_W_SCL(0);
		MCP4725_I2C_Delay();

		Byte <<= 1;
	}

	/* 第9个时钟，忽略ACK */
	MCP4725_W_SDA(1);
	MCP4725_I2C_Delay();
	MCP4725_W_SCL(1);
	MCP4725_I2C_Delay();
	MCP4725_W_SCL(0);
	MCP4725_I2C_Delay();
}

void MCP4725_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Pin = MCP4725_SCL_PIN | MCP4725_SDA_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	MCP4725_W_SCL(1);
	MCP4725_W_SDA(1);
}

/* 写DAC寄存器，不写EEPROM */
void MCP4725_SetValue(uint16_t Value)
{
	Value &= 0x0FFF;

	MCP4725_I2C_Start();
	MCP4725_I2C_SendByte(MCP4725_ADDR_WRITE);
	MCP4725_I2C_SendByte(0x40);                 /* 写DAC寄存器 */
	MCP4725_I2C_SendByte((uint8_t)(Value >> 4));
	MCP4725_I2C_SendByte((uint8_t)((Value & 0x000F) << 4));
	MCP4725_I2C_Stop();
}