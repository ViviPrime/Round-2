#include "stm32f10x.h"
#include "Motor.h"
#include "PWM.h"

// 电机1: AIN1-B12, AIN2-B13, PWMA-A2
// 电机2: BIN1-B14, BIN2-B15, PWMB-A3

void Motor_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 使能GPIOB和GPIOA时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOA, ENABLE);
    
    // 配置电机1方向引脚 B12, B13
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置电机2方向引脚 B14, B15
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 初始化PWM
    PWM_Init();
}

void Motor1_SetPWM(int16_t PWM) {
    if (PWM >= 0) {
        // 正转
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);  // AIN1 = 0
        GPIO_SetBits(GPIOB, GPIO_Pin_13);    // AIN2 = 1
        PWM_SetCompare1(PWM);                // PWMA
    } else {
        // 反转
        GPIO_SetBits(GPIOB, GPIO_Pin_12);    // AIN1 = 1
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);  // AIN2 = 0
        PWM_SetCompare1(-PWM);              // PWMA
    }
}

void Motor2_SetPWM(int16_t PWM) {
    if (PWM >= 0) {
        // 正转
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);  // BIN1 = 0
        GPIO_SetBits(GPIOB, GPIO_Pin_15);    // BIN2 = 1
        PWM_SetCompare2(PWM);                // PWMB
    } else {
        // 反转
        GPIO_SetBits(GPIOB, GPIO_Pin_14);    // BIN1 = 1
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);  // BIN2 = 0
        PWM_SetCompare2(-PWM);              // PWMB
    }
}
