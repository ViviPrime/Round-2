#include "Encoder.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

int32_t encoder1_position = 0;
int32_t encoder2_position = 0;
int16_t encoder1_speed = 0;
int16_t encoder2_speed = 0;
uint8_t encoder_data_ready = 0;

// 电机1编码器初始化 (A6, A7 -> TIM3_CH1, TIM3_CH2)
void Encoder1_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    // 配置GPIO A6, A7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置定时器时基
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    // 配置编码器接口
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, 
                              TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    // 配置输入捕获
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    
    // 使能定时器
    TIM_Cmd(TIM3, ENABLE);
}

// 电机2编码器初始化 (B6, B7 -> TIM4_CH1, TIM4_CH2)
void Encoder2_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    
    // 使能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // 配置GPIO B6, B7
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置定时器时基
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // 配置编码器接口
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, 
                              TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    
    // 配置输入捕获
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    
    // 使能定时器
    TIM_Cmd(TIM4, ENABLE);
}

// 定时器中断初始化 (10ms中断) - 使用TIM2替代TIM6
void Encoder_TIM_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使用TIM2替代TIM6（更通用的定时器）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    // 配置定时器时基
    // 72MHz / 7200 = 10kHz, 10kHz / 100 = 100Hz (10ms)
    TIM_TimeBaseStructure.TIM_Period = 100 - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    // 使能定时器更新中断
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    
    // 配置NVIC - 使用TIM2的中断通道
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  // TIM2的中断通道
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    // 使能定时器
    TIM_Cmd(TIM2, ENABLE);
}

// 读取编码器速度（在中断中调用）
void Encoder_ReadSpeed(void) {
    static int16_t last_count1 = 0;
    static int16_t last_count2 = 0;
    
    int16_t current_count1 = TIM_GetCounter(TIM3);
    int16_t current_count2 = TIM_GetCounter(TIM4);
    
    // 计算速度（考虑溢出）
    encoder1_speed = (int16_t)(current_count1 - last_count1);
    encoder2_speed = (int16_t)(current_count2 - last_count2);
    
    // 更新位置
    encoder1_position += encoder1_speed;
    encoder2_position += encoder2_speed;
    
    last_count1 = current_count1;
    last_count2 = current_count2;
    
    encoder_data_ready = 1;
}

int16_t Encoder1_GetSpeed(void) {
    return encoder1_speed;
}

int16_t Encoder2_GetSpeed(void) {
    return encoder2_speed;
}

int32_t Encoder1_GetPosition(void) {
    return encoder1_position;
}

int32_t Encoder2_GetPosition(void) {
    return encoder2_position;
}

uint8_t Encoder_DataReady(void) {
    return encoder_data_ready;
}

void Encoder_ClearDataFlag(void) {
    encoder_data_ready = 0;
}

void Encoder1_ClearPosition(void) {
    encoder1_position = 0;
    TIM_SetCounter(TIM3, 0);
}

void Encoder2_ClearPosition(void) {
    encoder2_position = 0;
    TIM_SetCounter(TIM4, 0);
}

// TIM2中断服务函数（替代TIM6）
void TIM2_IRQHandler(void) {
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
        Encoder_ReadSpeed();
    }
}
