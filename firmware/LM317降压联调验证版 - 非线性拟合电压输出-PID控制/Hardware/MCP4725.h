#ifndef __MCP4725_H
#define __MCP4725_H

#include "stm32f10x.h"

void MCP4725_Init(void);
void MCP4725_SetValue(uint16_t Value);

#endif