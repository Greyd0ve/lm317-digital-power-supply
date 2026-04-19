#ifndef __AD_H
#define __AD_H

#include "stm32f10x.h"

void AD_Init(void);
uint16_t AD_GetValue(uint8_t ADC_Channel);
uint16_t AD_GetAverage(uint8_t ADC_Channel, uint8_t Count);

#endif