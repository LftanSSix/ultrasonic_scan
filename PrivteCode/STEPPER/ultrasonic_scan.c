#include "ultrasonic_scan.h"
#include "usart.h"
#include "gpio.h"
extern TIM_HandleTypeDef htim2;

void Send_Pluse(void)
{

		HAL_GPIO_WritePin(trig_GPIO_Port,trig_Pin,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(trig_GPIO_Port,trig_Pin,GPIO_PIN_SET);
		HAL_Delay(1); //大于10us的触发脉冲
		HAL_GPIO_WritePin(trig_GPIO_Port,trig_Pin,GPIO_PIN_RESET);

}


//采用寄存器接收模式htim3.Instance->CNT里的值单位由分频器决定

float Receive(void)
{
	float t1,t2,distance;

	HAL_TIM_Base_Start_IT(&htim2);
	htim2.Instance->CNT = 0;

  while(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin) == GPIO_PIN_RESET)
	t1=htim2.Instance->CNT;


  while(HAL_GPIO_ReadPin(ECHO_GPIO_Port,ECHO_Pin) == GPIO_PIN_SET)
    t2=htim2.Instance->CNT;


	//340m/1s = 340mm/1ms = 0.34mm/1us =0.034cm/1us

  distance=(t2-t1)*0.017;
  return distance;
}
