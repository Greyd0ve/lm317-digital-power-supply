#include "stm32f10x.h"
#include "Delay.h"
#include "Key.h"

#define KEY_INC_PIN        GPIO_Pin_1
#define KEY_CONFIRM_PIN    GPIO_Pin_0
#define KEY_STEP_PIN       GPIO_Pin_10
#define KEY_DEC_PIN        GPIO_Pin_11

void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = KEY_INC_PIN | KEY_CONFIRM_PIN | KEY_STEP_PIN | KEY_DEC_PIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
   返回值：
   0 -> 无按键
   1 -> PB1按下   （电压增加）
   2 -> PB11按下  （电压减少）
   3 -> PB10按下  （步进切换）
   4 -> PB0按下   （确认设定）
*/
uint8_t Key_GetNum(void)
{
	static uint8_t KeyUp = 1;
	uint8_t KeyNum = 0;

	if (KeyUp &&
		(GPIO_ReadInputDataBit(GPIOB, KEY_INC_PIN) == 0 ||
		 GPIO_ReadInputDataBit(GPIOB, KEY_DEC_PIN) == 0 ||
		 GPIO_ReadInputDataBit(GPIOB, KEY_STEP_PIN) == 0 ||
		 GPIO_ReadInputDataBit(GPIOB, KEY_CONFIRM_PIN) == 0))
	{
		Delay_ms(20);
		KeyUp = 0;

		if (GPIO_ReadInputDataBit(GPIOB, KEY_INC_PIN) == 0)
		{
			KeyNum = 1;
		}
		else if (GPIO_ReadInputDataBit(GPIOB, KEY_DEC_PIN) == 0)
		{
			KeyNum = 2;
		}
		else if (GPIO_ReadInputDataBit(GPIOB, KEY_STEP_PIN) == 0)
		{
			KeyNum = 3;
		}
		else if (GPIO_ReadInputDataBit(GPIOB, KEY_CONFIRM_PIN) == 0)
		{
			KeyNum = 4;
		}
	}
	else if (GPIO_ReadInputDataBit(GPIOB, KEY_INC_PIN) == 1 &&
			 GPIO_ReadInputDataBit(GPIOB, KEY_DEC_PIN) == 1 &&
			 GPIO_ReadInputDataBit(GPIOB, KEY_STEP_PIN) == 1 &&
			 GPIO_ReadInputDataBit(GPIOB, KEY_CONFIRM_PIN) == 1)
	{
		KeyUp = 1;
	}

	return KeyNum;
}
