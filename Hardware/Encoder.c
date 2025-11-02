#include "Encoder.h"

TIM_HandleTypeDef htim2;  // 编码器定时器
int32_t encoder_position = 0;

void Encoder_Init(void) {
    TIM_Encoder_InitTypeDef encoder_config = {0};
    TIM_MasterConfigTypeDef master_config = {0};
    GPIO_InitTypeDef gpio_init = {0};
    
    // 时钟使能
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // GPIO配置 - PA0, PA1 (TIM2_CH1, TIM2_CH2)
    gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio_init.Mode = GPIO_MODE_INPUT;
    gpio_init.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &gpio_init);
    
    // 定时器基础配置
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    
    // 编码器模式配置
    encoder_config.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder_config.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder_config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder_config.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder_config.IC1Filter = 0;
    encoder_config.IC2Filter = 0;
    
    HAL_TIM_Encoder_Init(&htim2, &encoder_config);
    
    // 主模式配置
    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_config);
    
    // 启动编码器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
}

int16_t Encoder_GetSpeed(void) {
    static int16_t last_count = 0;
    int16_t current_count = (int16_t)TIM2->CNT;
    int16_t speed = current_count - last_count;
    last_count = current_count;
    return speed;
}

int32_t Encoder_GetPosition(void) {
    return encoder_position;
}

void Encoder_ClearPosition(void) {
    encoder_position = 0;
    __HAL_TIM_SET_COUNTER(&htim2, 0);
}