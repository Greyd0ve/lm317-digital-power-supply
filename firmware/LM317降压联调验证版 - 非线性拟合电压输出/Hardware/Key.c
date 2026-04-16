#include "stm32f10x.h"
#include "Delay.h"
#include "Key.h"

void Key_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
   返回值：
   0 -> 无按键
   1 -> PB1按下   （电压增加）
   2 -> PB11按下  （电压减少）
   3 -> PB10按下  （步进切换）
*/
uint8_t Key_GetNum(void)
{
	static uint8_t KeyUp = 1;
	uint8_t KeyNum = 0;

	if (KeyUp &&
		(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0 ||
		 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0 ||
		 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0))
	{
		Delay_ms(20);
		KeyUp = 0;

		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
		{
			KeyNum = 1;
		}
		else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 0)
		{
			KeyNum = 2;
		}
		else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 0)
		{
			KeyNum = 3;
		}
	}
	else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 1 &&
			 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10) == 1 &&
			 GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1)
	{
		KeyUp = 1;
	}

	return KeyNum;
}