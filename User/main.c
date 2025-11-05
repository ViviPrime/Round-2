#include "stm32f10x.h"
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include <stdlib.h>

// PID控制变量
float Target, Actual, Out;		
float Kp, Ki, Kd;  // PID参数
float Error0 = 0, Error1 = 0, Error2 = 0;

// 系统状态变量
uint8_t p=0;
uint8_t KeyNum;

static uint32_t tickCount = 0;

uint32_t GetTickCount(void)
{
    return tickCount;
}

int main(void)
{
    // 系统初始化
    OLED_Init();          // 只初始化一次
    Motor_Init();	
    Encoder1_Init();
    Encoder2_Init();		
    Serial_Init();
    Timer_Init();
    Key_Init();
    // 显示初始信息
    OLED_Printf(0, 0, OLED_8X16, "Speed Control");
    OLED_Update();
    
    Target = 0;  // 初始目标速度为0
    Kp=5, Ki=1, Kd=3;
	
    while (1)
	{
		KeyNum = Key_GetNum();
		if(KeyNum==1)
		{
			static uint32_t lastKeyTime = 0;  // 新增：记录上次按键时间
            uint32_t currentTime = GetTickCount();
            
            // 防抖处理：30ms内只响应一次按键
            if(currentTime - lastKeyTime > 30)
            {
                Target=0;
			    Actual=0;
			    Out = 0;
			    Error0 = 0;
			    Error1 = 0;
			    Error2 = 0;

			    p=1-p; 
				if(p == 1) {  // 切换到模式2时
                   Encoder1_ClearPosition();
                   Encoder2_ClearPosition();
                }
				
			    OLED_Clear();
			    OLED_Update();
                lastKeyTime = currentTime;
            }
			
		}
		if(p==0)
		{	
			Kp=5, Ki=2, Kd=3;
			OLED_Printf(0, 0, OLED_8X16, "Speed Control");
			OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);	
			OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);	
			OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);	
			OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
			OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
			OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);	
			OLED_Update();
		}
		else{
			Kp=3, Ki=0.1 , Kd=0;
			OLED_Printf(0, 0, OLED_8X16, "Location Control");
			OLED_Printf(0, 16, OLED_8X16, "Kp:%4.2f", Kp);	
			OLED_Printf(0, 32, OLED_8X16, "Ki:%4.2f", Ki);	
			OLED_Printf(0, 48, OLED_8X16, "Kd:%4.2f", Kd);
			OLED_Printf(64, 16, OLED_8X16, "Tar:%+04.0f", Target);
			OLED_Printf(64, 32, OLED_8X16, "Act:%+04.0f", Actual);
			OLED_Printf(64, 48, OLED_8X16, "Out:%+04.0f", Out);	
			OLED_Update();
		}
		Serial_Printf("%f,%f,%f\r\n", Target, Actual, Out);	
		if(Serial_RxFlag == 1)
		{
			Target = (float)atof(Serial_RxPacket);
			Serial_RxFlag = 0;
		}
	}
}

void TIM1_UP_IRQHandler(void)
{
    static uint16_t Count;	
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        tickCount++;
        Count++;	
        Key_Tick();	
        
        if (Count >= 10)	
        {
            Count = 0;	
            
            if(p == 0){  // 模式1：速度控制
                // 电机1速度PID控制
                Actual = Encoder1_GetSpeed();
                Error2 = Error1;
                Error1 = Error0;	
                Error0 = Target - Actual;	
                Out += Kp * (Error0 - Error1) + Ki * Error0 + Kd * (Error0 - 2 * Error1 + Error2);
                if (Out > 100) Out = 100;	
                if (Out < -100) Out = -100;
                Motor1_SetPWM(Out);
                Motor2_SetPWM(0);  // 电机2停止
            }
            else{  // 模式2：位置跟随控制
                static int32_t LastPosition1 = 0;
                static int32_t BasePosition2 = 0;
                static uint8_t FirstTime = 1;
                
                // 第一次进入模式2时初始化
                if (FirstTime) {
                    LastPosition1 = Encoder1_GetPosition();
                    BasePosition2 = Encoder2_GetPosition();
                    FirstTime = 0;
                    Error0 = Error1 = Error2 = 0;  // 重置PID误差
                    Out = 0;  // 重置输出
                }
                
                // 获取当前位置
                int32_t CurrentPosition1 = Encoder1_GetPosition();
                int32_t CurrentPosition2 = Encoder2_GetPosition();
                
                // 计算电机1的位置变化量
                int32_t DeltaPosition1 = CurrentPosition1 - LastPosition1;
                LastPosition1 = CurrentPosition1;
                
                // 设置电机2的目标位置 = 基础位置 + 电机1的位移
                int32_t TargetPosition2 = BasePosition2 + DeltaPosition1;
                
                // 电机2位置PID控制
                int32_t PositionError = TargetPosition2 - CurrentPosition2;
                
                Error2 = Error1;
                Error1 = Error0;	
                Error0 = (float)PositionError;  // 转换为float进行PID计算
                
                Out += Kp * (Error0 - Error1) + Ki * Error0 + Kd * (Error0 - 2 * Error1 + Error2);
                
                if (Out > 100) Out = 100;	
                if (Out < -100) Out = -100;
                
                Motor2_SetPWM(Out);  // 控制电机2跟随
                Motor1_SetPWM(0);   // 电机1自由转动（手动控制）
            }
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}


