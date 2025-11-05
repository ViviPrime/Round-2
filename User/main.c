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
		Count ++;
		Key_Tick();	
		if (Count >= 10)	
		{
			Count = 0;	
			if(p==0){
				Actual = Encoder1_GetSpeed();
				Error2 = Error1;
				Error1 = Error0;	
				Error0 = Target - Actual;	
				Out += Kp * (Error0 - Error1) + Ki * Error0
					+ Kd * (Error0 - 2 * Error1 + Error2);
				if (Out > 100) {Out = 100;}	
				if (Out < -100) {Out = -100;}
				Motor1_SetPWM(Out);
			}
			else
			{
				Target+=Encoder1_GetSpeed();
				Actual+=Encoder2_GetSpeed();
				Error2 = Error1;
				Error1 = Error0;	
				Error0 = Target - Actual;	
				Out += Kp * (Error0 - Error1) + Ki * Error0
					+ Kd * (Error0 - 2 * Error1 + Error2);
				if (Out > 100) {Out = 100;}	
				if (Out < -100) {Out = -100;}
				Motor2_SetPWM(Out);
			}
		}
		
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
}


