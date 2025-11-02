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

// 全局变量用于数据上传
int16_t current_speed1 = 0, current_speed2 = 0;
int32_t current_position1 = 0, current_position2 = 0;
uint8_t current_mode = 0;

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
    Encoder1_Init();
    Encoder2_Init();
    Motor_Init();
    Key_Init();
    Serial_Init();
    OLED_Init();
    
    // 初始化10ms定时器中断用于编码器数据读取
    Encoder_TIM_Init();
    
    // 速度PID参数初始化
    speed_pid1.Kp = 0.8; speed_pid1.Ki = 0.1; speed_pid1.Kd = 0.05;
    speed_pid2.Kp = 0.8; speed_pid2.Ki = 0.1; speed_pid2.Kd = 0.05;
    speed_pid1.integral_limit = speed_pid2.integral_limit = 1000;
    speed_pid1.integral = speed_pid2.integral = 0;
    speed_pid1.last_error = speed_pid2.last_error = 0;
    
    // 位置PID参数初始化
    position_pid1.Kp = 1.0; position_pid1.Ki = 0.01; position_pid1.Kd = 0.1;
    position_pid2.Kp = 1.0; position_pid2.Ki = 0.01; position_pid2.Kd = 0.1;
    position_pid1.integral_limit = position_pid2.integral_limit = 500;
    position_pid1.integral = position_pid2.integral = 0;
    position_pid1.last_error = position_pid2.last_error = 0;
    
    float target_speed1 = 0, target_speed2 = 0;
    int32_t target_position2 = 0;
    uint8_t mode = 0; // 0:速度模式 1:位置随动模式
    uint8_t last_key = 0;
    uint32_t display_counter = 0;
    
    OLED_Clear();
    OLED_ShowString(1, 1, "DC Motor Control");
    OLED_ShowString(2, 1, "Mode:Speed");
    Serial_Printf("System Initialized\r\n");
    Serial_Printf("Data Format: S1:speed1,S2:speed2,P1:position1,P2:position2,M:mode\r\n");
    
    while (1) {
        // 检查编码器数据是否准备好（10ms中断更新）
        if (Encoder_DataReady()) {
            // 获取最新的编码器数据
            current_speed1 = Encoder1_GetSpeed();
            current_speed2 = Encoder2_GetSpeed();
            current_position1 = Encoder1_GetPosition();
            current_position2 = Encoder2_GetPosition();
            current_mode = mode;
            
            // 通过串口发送数据到上位机
            Serial_Printf("S1:%d,S2:%d,P1:%ld,P2:%ld,M:%d\r\n", 
                         current_speed1, 
                         current_speed2,
                         current_position1,
                         current_position2,
                         current_mode);
            
            // 清除数据标志
            Encoder_ClearDataFlag();
        }
        
        float pwm_output1, pwm_output2;
        
        if (mode == 0) {
            // 速度模式
            pwm_output1 = PID_Calculate(&speed_pid1, target_speed1, current_speed1);
            pwm_output2 = PID_Calculate(&speed_pid2, target_speed2, current_speed2);
            
            // 显示更新（降低刷新频率避免OLED闪烁）
            if (display_counter % 10 == 0) {
                OLED_ShowString(2, 1, "Mode:Speed      ");
                OLED_ShowString(3, 1, "M1:");
                OLED_ShowSignedNum(3, 4, current_speed1, 5);
                OLED_ShowString(4, 1, "M2:");
                OLED_ShowSignedNum(4, 4, current_speed2, 5);
            }
        } else {
            // 位置随动模式
            target_position2 = current_position1; // 电机2跟随电机1位置
            pwm_output1 = 0; // 电机1自由转动
            pwm_output2 = PID_Calculate(&position_pid2, target_position2, current_position2);
            
            // 显示更新
            if (display_counter % 10 == 0) {
                OLED_ShowString(2, 1, "Mode:Position   ");
                OLED_ShowString(3, 1, "Pos1:");
                OLED_ShowSignedNum(3, 6, current_position1, 6);
                OLED_ShowString(4, 1, "Pos2:");
                OLED_ShowSignedNum(4, 6, current_position2, 6);
            }
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
            OLED_ShowString(1, 1, "DC Motor Control");
            Serial_Printf("Mode changed: %s\r\n", mode ? "Position" : "Speed");
        }
        last_key = key;
        
        // 串口指令处理
        if (Serial_GetRxFlag()) {
            uint8_t data = Serial_GetRxData();
            if (data == 's' || data == 'S') {
                target_speed1 = 30;
                target_speed2 = 30;
                Serial_Printf("Set speed to 30\r\n");
            } else if (data == 't' || data == 'T') {
                target_speed1 = 0;
                target_speed2 = 0;
                Serial_Printf("Stop motors\r\n");
            } else if (data == 'c' || data == 'C') {
                Encoder1_ClearPosition();
                Encoder2_ClearPosition();
                Serial_Printf("Position cleared\r\n");
            } else if (data == '?') {
                Serial_Printf("Commands: s-start, t-stop, c-clear, ?-help\r\n");
            }
        }
        
        display_counter++;
        Delay_ms(1); // 减少延时，提高响应速度
    }
}
