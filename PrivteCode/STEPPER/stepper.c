#include "stepper.h"
#include "gpio.h"

//IN1: PB6 a
//IN2: PB7  b
//IN3: PB8  c
//IN4: PB9  d

uint8_t forward[4] = {0x03,0x06,0x0c,0x09}; // 正转
uint8_t reverse[4]= {0x03,0x09,0x0c,0x06}; // 反转

MachineState motor_state = {0};

//根据输入数据改变引脚电平
void SetMotor(unsigned char InputData)
{
	if(InputData == 0x03)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

	}
	else if(InputData == 0x06)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	else if(InputData == 0x09)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	}
	else if(InputData == 0x0c)
	{	
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	}
	else if(InputData == 0x00)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	}
}

/*
	功能：转1/64圈
	步距角5.625 360/5.625=64 减速比1/64
	64*64=4096个脉冲

	direction 方向 1CW 0CCW
	delay delay时长ms >= 2
*/
void motor_circle(int angle, int direction, int delay)
{
    int i, j, n;
    n=(int)(angle*512/360);
    for(i = 0; i < n; i++)
    {
		for(j = 0; j < 4; j++)
		{
			if(1 == direction)
			{
				//SetMotor(0x00);
				SetMotor(forward[j]);
				HAL_Delay(delay);
			}
			else
			{
				//SetMotor(0x00);
				SetMotor(reverse[j]);
				HAL_Delay(delay);
			}


		}
    }
}

//连续转动函数
void Motor_contin(int direction, int delay)
{
	int j;
	for(j = 0; j < 4; j++)
			{
				if(1 == direction)
				{
					//SetMotor(0x00);
					SetMotor(forward[j]);
					HAL_Delay(delay);
				}
				else
				{
					//SetMotor(0x00);
					SetMotor(reverse[j]);
					HAL_Delay(delay);
				}


			}
}

////Find 0 point
////HomeKey中断回调函数
//
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	if(GPIO_Pin == home_key_Pin)
//	{
//		motor_state.home_state = 0;
//		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
//		//HAL_Delay(5000);//外部中断优先级不能高于systick (0,0)
//		while(HAL_GPIO_ReadPin(GPIOA, limit_key_Pin) == GPIO_PIN_SET)
//		{
//			Motor_contin(1,2);
//		}
//		//找到零位并移动至待机点
//		SetMotor(0);
//		HAL_Delay(20);
//		motor_circle(100, 0, 2);
//		motor_state.home_state = 1; //回家完成状态更新
//
//		//HAL_EXTI_GenerateSWI();//生成软件中断
//
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(2000);
//		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//		HAL_Delay(2000);
//
//		//HAL_EXTI_ClearPending(&EXTI_H_KEY, EXTI_TRIGGER_FALLING);
//
//		__HAL_GPIO_EXTI_CLEAR_FLAG(home_key_Pin);
//		__HAL_GPIO_EXTI_CLEAR_IT(home_key_Pin); //清标志位home
//		__HAL_GPIO_EXTI_CLEAR_IT(limit_key_Pin); //清标志位limit
//	}
//
//	if(GPIO_Pin == limit_key_Pin)
//	{
//		//运行中撞限位处理
//		//LimitKey中断回调函数
//		//uint8_t checkstate;
//		//checkstate = motor_state.home_state;
//		if(motor_state.home_state == 1)
//		{
//			SetMotor(0);
//			while(1)
//			{
//				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//				HAL_Delay(5000);
//			}
//
//		}else
//			__HAL_GPIO_EXTI_CLEAR_IT(limit_key_Pin);
//
//
//	}
//}


















