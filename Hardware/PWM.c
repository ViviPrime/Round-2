#include "stm32f10x.h"
#include "PWM.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

// PWMA -> A2 (TIM2_CH3)
// PWMB -> A3 (TIM2_CH4)

void PWM_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // 使能时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置GPIO A2, A3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置定时器时基
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;      // PWM周期
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;   // 72MHz/72 = 1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 配置PWM通道3 (A2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);
    
    // 配置PWM通道4 (A3)
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    
    // 使能定时器
    TIM_Cmd(TIM2, ENABLE);
    
    // 使能PWM输出
    TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void PWM_SetCompare1(uint16_t Compare) {
    TIM_SetCompare3(TIM2, Compare);
}

void PWM_SetCompare2(uint16_t Compare) {
    TIM_SetCompare4(TIM2, Compare);
}
