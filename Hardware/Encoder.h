#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void Encoder1_Init(void);
void Encoder2_Init(void);
int16_t Encoder1_GetSpeed(void);
int16_t Encoder2_GetSpeed(void);

#endif
