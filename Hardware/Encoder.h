#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void Encoder1_Init(void);
void Encoder2_Init(void);
void Encoder_TIM_Init(void);
int16_t Encoder1_GetSpeed(void);
int16_t Encoder2_GetSpeed(void);
int32_t Encoder1_GetPosition(void);
int32_t Encoder2_GetPosition(void);
uint8_t Encoder_DataReady(void);
void Encoder_ClearDataFlag(void);
void Encoder1_ClearPosition(void);
void Encoder2_ClearPosition(void);

#endif
