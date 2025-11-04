#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include <stdlib.h>
#include <string.h>

// PID控制变量
float Target = 0, Actual = 0, Out = 0;		
float Kp = 5.0f, Ki = 1.0f, Kd = 2.0f;  // 默认PID参数
float Error0 = 0, Error1 = 0, Error2 = 0;

// 系统状态变量
uint32_t SystemTick = 0;
uint8_t UpdateDisplay = 1;


int main(void)
{
    // 系统初始化
    OLED_Init();          // 只初始化一次
    Motor_Init();	
    Encoder1_Init();
    Encoder2_Init();		
    Serial_Init();
    Timer_Init();
    
    // 显示初始信息
    OLED_Printf(0, 0, OLED_8X16, "Speed Control");
    OLED_Update();
    
    Target = 0;  // 初始目标速度为0
    
    while (1)
    {
        // 每100ms更新一次显示，降低CPU占用
        if(SystemTick % 10 == 0 && UpdateDisplay)
        {
            OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);	
            OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);	
            OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);	
            
            OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
            OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
            OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);	
            
            OLED_Update();
            UpdateDisplay = 0;  // 重置显示标志
        }
        
        // 串口数据发送（可降低频率）
        if(SystemTick % 20 == 0)
        {
            Serial_Printf("%.2f,%.2f,%.2f\r\n", Target, Actual, Out);	
        }
        
        // 处理串口接收
        if(Serial_RxFlag == 1)
        {
            // 验证数据有效性
            if(strlen(Serial_RxPacket) > 0 && strlen(Serial_RxPacket) < 50)
            {
                float newTarget = (float)atof(Serial_RxPacket);
                // 限制目标速度范围
                if(newTarget <= 1000 && newTarget >= -1000)
                {
                    Target = newTarget;
                }
            }
            Serial_RxFlag = 0;
            UpdateDisplay = 1;  // 更新显示
        }
    }
}

// 优化后的定时器中断服务函数
void TIM1_UP_IRQHandler(void)
{
    static uint16_t Count = 0;	
    
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        Count++;
        SystemTick++;
        
        // 每10ms执行一次PID控制（100Hz）
        if (Count >= 1)	
        {
            Count = 0;	
            
            // 读取编码器速度并转换类型
            Actual = (float)Encoder1_GetSpeed();
            
            // 更新误差
            Error2 = Error1;
            Error1 = Error0;	
            Error0 = Target - Actual;	
            
            // 标准位置式PID算法
            float P_Term = Kp * Error0;
            float I_Term = Ki * (Error0 + Error1 + Error2) * 0.001f;  // 乘以时间因子
            float D_Term = Kd * (Error0 - Error1) * 100.0f;           // 乘以时间因子的倒数
            
            Out = P_Term + I_Term + D_Term;
            
            // 输出限幅
            if (Out > 100.0f) Out = 100.0f;	
            if (Out < -100.0f) Out = -100.0f;
            
            Motor1_SetPWM((int16_t)Out);  // 转换为整数输出
            
            // 每100ms允许更新显示
            if(SystemTick % 10 == 0)
            {
                UpdateDisplay = 1;
            }
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
