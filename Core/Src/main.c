/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stepper.h"
#include "ultrasonic_scan.h"
#include "delay.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
UART_HandleTypeDef huart1;
MachineState motor_state;
set_workmode workmode = {1};//默认自动模式

//extern uint8_t aRxBuffer[RXBUFFERSIZE];
//extern uint8_t USART_RX_BUF[USART_REC_LEN];
//extern uint16_t USART_RX_STA;
//重定向printf函数
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    //具体哪个串口可以更改huart1
    HAL_UART_Transmit(&huart1 , (uint8_t *)&ch, 1 , 0xffff);
    return ch;
}
//重定向scanf函数
int fgetc(FILE *stream)
{
	uint8_t ch[1];
	HAL_UART_Receive(&huart1,ch,1,0xFFFF);
	return ch[0];
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t hal_Rx_buff[1] = {0}; //接收中断-缓冲�???
uint8_t uart1_Rx_buff[10] = {0};

static uint32_t T_start,T_stop,HighTime;
static float target_dis = 0;
int angle_num = 361-1; //250°*1.44=361
static int scan_count;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);//设置滴答计时器中断优先级为最高�?�防止HAL_Delay在中断中卡死
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, hal_Rx_buff, 1);//重新�???启接收中�???,???放在这里和放在WHILE循环中有�???么区�???
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_TogglePin(D3_GPIO_Port, D3_Pin);
	  switch(scan_count)

	  {
	  	  case 361:
			  motor_state.in_halfposition = 1; //完成半轮扫描。一个来回为一轮
			  angle_num = 361-1;
			  __HAL_GPIO_EXTI_GENERATE_SWIT(RunAndScan_Pin); //第二次顺时针扫描
			  //Send_Pluse(); //第二次顺时针扫描
			  break;
		  case 722:
			  motor_state.in_position = 1; //完成一轮扫描，准备第二轮扫描
			  motor_state.in_halfposition = 0;
			  scan_count = 0;
			  angle_num = 361-2;
			  break;

	  }
//	  Send_Pluse();
//	  HAL_Delay(300);


	  switch(uart1_Rx_buff[0])
	  {
	  	  case 0x01:
			  printf("hal_Rx_buff: %d\r\n",uart1_Rx_buff[0]);
			  __HAL_GPIO_EXTI_GENERATE_SWIT(home_key_Pin);
			  uart1_Rx_buff[0] = 0; //0X01指令�???次触发一次回�???
			  break;
	  	  case 0x02:
			  printf("hal_Rx_buff: %d\r\n",uart1_Rx_buff[0]);
			  if(motor_state.home_state == 1)workmode.mode = 0; //�???启手动模式，禁用自动模式
			  uart1_Rx_buff[0] = 0;
			  break;
	  	  case 0x03:
	  		  if(workmode.mode == 0)
	  		  {
				  motor_state.jog_dir = cw; //顺时针转5�???
				  printf("hal_Rx_buff: %d\r\n",uart1_Rx_buff[0]);
				  __HAL_GPIO_EXTI_GENERATE_SWIT(RunAndScan_Pin);
				  uart1_Rx_buff[0] = 0;
				  break;
	  		  }

	  	  case 0x04:
	  		  if(workmode.mode == 0)
	  		  {
		  		  motor_state.jog_dir = ccw; //逆时针转5�???
				  printf("hal_Rx_buff: %d\r\n",uart1_Rx_buff[0]);
				  __HAL_GPIO_EXTI_GENERATE_SWIT(RunAndScan_Pin);
				  uart1_Rx_buff[0] = 0;
				  break;
	  		  }
	  	  case 0x05:
			  printf("hal_Rx_buff: %d\r\n",uart1_Rx_buff[0]);
			  if(motor_state.home_state == 1)workmode.mode = 1; //退出手动模式
			  uart1_Rx_buff[0] = 0;
			  break;

	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* USART1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 3);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */

//Find 0 point
//HomeKey中断回调函数
float juli=0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == home_key_Pin)
	{
		motor_state.home_state = 0;
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, GPIO_PIN_RESET);
		//HAL_Delay(5000);//外部中断优先级不能高于systick (0,0)
		while(HAL_GPIO_ReadPin(GPIOA, limit_key_Pin) == GPIO_PIN_SET)
		{
			Motor_contin(1,2);
		}
		//找到零位并移动至待机
		SetMotor(0);
		HAL_Delay(20);
		motor_circle(10, 0, 2);  //80正对
		motor_state.home_state = 1; //回家完成状态更新

		__HAL_GPIO_EXTI_CLEAR_IT(home_key_Pin); //清标志位home
		__HAL_GPIO_EXTI_CLEAR_IT(limit_key_Pin); //清标志位limit
	}

	if(GPIO_Pin == limit_key_Pin)
	{
		//运行中撞限位处理
		//LimitKey中断回调函数
		//uint8_t checkstate;
		//checkstate = motor_state.home_state;
		if(motor_state.home_state == 1)
		{
			SetMotor(0);
			while(1)
			{
				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_Delay(5000);
			}
			__HAL_GPIO_EXTI_CLEAR_IT(limit_key_Pin);
		}else
			__HAL_GPIO_EXTI_CLEAR_IT(limit_key_Pin);


	}

	if(GPIO_Pin == RunAndScan_Pin)
	{
/*******自动模式*******/

		//RunAndScan中断回调函数
		uint8_t i;

		if(workmode.mode == 1)
		{
			if(motor_state.home_state == 1)
			{

					if(motor_state.in_halfposition == 0)
					{
						motor_circle(1, ccw, 2);
						Send_Pluse();
						__HAL_GPIO_EXTI_CLEAR_IT(RunAndScan_Pin);
					}else
					{
						motor_circle(1, cw, 2);
						Send_Pluse();
						__HAL_GPIO_EXTI_CLEAR_IT(RunAndScan_Pin);
					}



			}else
				__HAL_GPIO_EXTI_CLEAR_IT(RunAndScan_Pin);
		}

/*******手动模式*******/
		if(EXTI->SWIER & 0X04 ) //判断是软件中断
		{
			if(workmode.mode == 0)
				{
				switch(motor_state.jog_dir)
				   {case(cw):
						for(i=0;i<7;i++)//CW顺时针扫描5度， 5*1.44。 1.44实际角度系数
						{
							motor_circle(1, cw, 2);
						}i=0;
						__HAL_GPIO_EXTI_CLEAR_IT(RunAndScan_Pin);
						break;
					case(ccw):
						for(i=0;i<7;i++)//CCW顺时针扫描
						{
							motor_circle(1, ccw, 2);
						}i=0;
						__HAL_GPIO_EXTI_CLEAR_IT(RunAndScan_Pin);
						break;
					}
				}

		}

	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		printf("You send data : \r\n");
		HAL_UART_Receive_IT(&huart1, hal_Rx_buff, 1);
		uart1_Rx_buff[0] = hal_Rx_buff[0];

		HAL_UART_Transmit(&huart1,hal_Rx_buff,1,0xFFFF);
	}

//	if(huart->Instance==USART1)
//	{
//		if((USART_RX_STA&0x8000)==0)
//		{
//			if(USART_RX_STA&0x4000)//收到0X0D
//			{
//				if(aRxBuffer[0]!=0x0a)USART_RX_STA=0;//未收�???0X0A,USART_RX_STA状�?�重�???
//				else
//				{
//					USART_RX_STA|=0x8000;//收到0X0A,接收到回车信号（0D�???0A），传输完成
//					if(USART_RX_BUF[0]== 0x01)//0x01--home信号
//					{
//						printf("buf[0]=%d \r\n",USART_RX_BUF[0]);
//						//HAL_EXTI_GenerateSWI();
//						__HAL_GPIO_EXTI_GENERATE_SWIT(home_key_Pin);
//					}
//
//				}
//
//
//			}
//			else
//			{
//				if(aRxBuffer[0]==0x0d)USART_RX_STA|=0x4000;
//				else//未收�???0X0D
//				{
//					USART_RX_BUF[USART_RX_STA&0X3FFF]=aRxBuffer[0] ;//数据储存至USART_RX_BUF
//					USART_RX_STA++;
//					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;
//				}
//			}
//		}
//
//	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

	//HAL_Delay(100);
	Send_Pluse();
	//printf("Z1 \r\n");
	if(htim2.Instance==TIM2)
	{

					T_start = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4); //获取上升沿时间点
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING); //切换下降沿捕获
					HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);//在中断中重新启动输入捕获切换捕获极性后，重新启动
					HAL_Delay(2);
					//printf("d1 = %d cm \r\n", (int)T_start);

					T_stop = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4);//获取下降沿时间点
					__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING); //切换上升沿捕获
					HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4);//切换捕获极性后，重新启动
					//HAL_Delay(2);
					//printf("d2 = %d cm \r\n", (int)T_stop);

				//高电平持续时间 = 下降沿时间点 - 上升沿时间点
				HighTime =T_stop - T_start;
				target_dis = HighTime * 0.17 ; //计算超声波测量距离mm
				scan_count++;
				printf("target_dis = %f mm \r\n", target_dis);
				//printf("count = %d  \r\n", scan_count);
				if(angle_num != 0 &&motor_state.in_position != 1)
				{
					HAL_Delay(1);
					__HAL_GPIO_EXTI_GENERATE_SWIT(RunAndScan_Pin);
					angle_num--;
				}else
					motor_state.in_position = 0 ;

	}
}

/**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
