#include "stm32f10x.h"
#include "Encoder.h"
#include "Motor.h"
#include "Key.h"
#include "OLED.h"
#include "Serial.h"
#include "Delay.h"
#include <stdio.h>

// PID参数结构体
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float last_error;
    float integral_limit;
} PID_TypeDef;

PID_TypeDef speed_pid1, speed_pid2;
PID_TypeDef position_pid1, position_pid2;

// PID计算函数
float PID_Calculate(PID_TypeDef *pid, float target, float current) {
    float error = target - current;
    
    // 积分限幅
    pid->integral += error;
    if(pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    
    float derivative = error - pid->last_error;
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    pid->last_error = error;
    
    return output;
}

int main(void) {
    // 系统初始化
    SystemInit();
    Encoder1_Init();
    Encoder2_Init();
    Motor_Init();
    Key_Init();
    Serial_Init();
    OLED_Init();
    
    // 速度PID参数初始化
    speed_pid1.Kp = 0.8; speed_pid1.Ki = 0.1; speed_pid1.Kd = 0.05;
    speed_pid2.Kp = 0.8; speed_pid2.Ki = 0.1; speed_pid2.Kd = 0.05;
    speed_pid1.integral_limit = speed_pid2.integral_limit = 1000;
    
    // 位置PID参数初始化
    position_pid1.Kp = 1.0; position_pid1.Ki = 0.01; position_pid1.Kd = 0.1;
    position_pid2.Kp = 1.0; position_pid2.Ki = 0.01; position_pid2.Kd = 0.1;
    position_pid1.integral_limit = position_pid2.integral_limit = 500;
    
    float target_speed1 = 0, target_speed2 = 0;
    int32_t target_position2 = 0;
    uint8_t mode = 0; // 0:速度模式 1:位置随动模式
    uint8_t last_key = 0;
    
    OLED_ShowString(0, 0, "DC Motor Control");
    Serial_Printf("System Initialized\r\n");
    
    while (1) {
        // 读取编码器
        int16_t current_speed1 = Encoder1_GetSpeed();
        int16_t current_speed2 = Encoder2_GetSpeed();
        int32_t current_position1 = Encoder1_GetPosition();
        int32_t current_position2 = Encoder2_GetPosition();
        
        float pwm_output1, pwm_output2;
        
        if (mode == 0) {
            // 速度模式
            pwm_output1 = PID_Calculate(&speed_pid1, target_speed1, current_speed1);
            pwm_output2 = PID_Calculate(&speed_pid2, target_speed2, current_speed2);
            
            OLED_ShowString(0, 2, "Speed Mode");
            OLED_ShowString(0, 4, "M1:");
            OLED_ShowNum(24, 4, current_speed1, 5);
            OLED_ShowString(0, 6, "M2:");
            OLED_ShowNum(24, 6, current_speed2, 5);
        } else {
            // 位置随动模式
            target_position2 = current_position1; // 电机2跟随电机1位置
            pwm_output1 = 0; // 电机1自由转动
            pwm_output2 = PID_Calculate(&position_pid2, target_position2, current_position2);
            
            OLED_ShowString(0, 2, "Position Mode");
            OLED_ShowString(0, 4, "Pos1:");
            OLED_ShowNum(40, 4, current_position1, 6);
            OLED_ShowString(0, 6, "Pos2:");
            OLED_ShowNum(40, 6, current_position2, 6);
        }
        
        // 限制PWM输出范围
        if (pwm_output1 > 100) pwm_output1 = 100;
        if (pwm_output1 < -100) pwm_output1 = -100;
        if (pwm_output2 > 100) pwm_output2 = 100;
        if (pwm_output2 < -100) pwm_output2 = -100;
        
        // 电机控制
        Motor1_SetPWM((int16_t)pwm_output1);
        Motor2_SetPWM((int16_t)pwm_output2);
        
        // 按键处理
        uint8_t key = Key_GetNum();
        if (key == 1 && last_key == 0) {
            mode = !mode;
            // 切换模式时清除积分项
            speed_pid1.integral = speed_pid2.integral = 0;
            position_pid1.integral = position_pid2.integral = 0;
            OLED_Clear();
            Serial_Printf("Mode changed: %s\r\n", mode ? "Position" : "Speed");
        }
        last_key = key;
        
        // 串口指令处理
        if (Serial_GetRxFlag()) {
            uint8_t data = Serial_GetRxData();
            if (data == 's') {
                target_speed1 = 30;
                target_speed2 = 30;
                Serial_Printf("Set speed to 30\r\n");
            } else if (data == 'S') {
                target_speed1 = 0;
                target_speed2 = 0;
                Serial_Printf("Stop motors\r\n");
            }
        }
        
        // 串口输出状态
        static uint32_t counter = 0;
        if (counter++ >= 100) {
            counter = 0;
            if (mode == 0) {
                Serial_Printf("Speed - M1:%d M2:%d\r\n", current_speed1, current_speed2);
            } else {
                Serial_Printf("Position - M1:%ld M2:%ld\r\n", current_position1, current_position2);
            }
        }
        
        Delay_ms(10);
    }
}
