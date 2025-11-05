#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_Init(void);
void Motor1_SetPWM(int8_t PWM);
void Motor2_SetPWM(int8_t PWM);

#endif
