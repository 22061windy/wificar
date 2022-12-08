/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "string.h"

#include "Legacy/stm32_hal_legacy.h"
#define USE_HAL_LEGACY

#define Echo_GPIO GPIOC		
#define Echo_PIN GPIO_PIN_2

#define Trig_GPIO GPIOC		
#define Trig_PIN GPIO_PIN_3

//定义电机pwm占空比
#define full_speed 150

#define null_speed 0

#define correct_speed 100

#define MOTOR_A_CON1_GPIO GPIOA
#define MOTOR_A_CON1_PIN GPIO_PIN_2
	 
#define MOTOR_A_CON2_GPIO GPIOB
#define MOTOR_A_CON2_PIN GPIO_PIN_9
	 
#define MOTOR_A_EN_GPIO GPIOA
#define MOTOR_A_EN_PIN GPIO_PIN_0

	 
#define MOTOR_B_CON1_GPIO GPIOA
#define MOTOR_B_CON1_PIN GPIO_PIN_1
	 
#define MOTOR_B_CON2_GPIO GPIOC
#define MOTOR_B_CON2_PIN GPIO_PIN_9
	 
#define MOTOR_B_EN_GPIO GPIOA
#define MOTOR_B_EN_PIN GPIO_PIN_3

//定义红外引脚
#define Left1_GPIO GPIOC	 
#define Left1_PIN GPIO_PIN_7

#define Left2_GPIO GPIOC	 
#define Left2_PIN GPIO_PIN_6	
 
#define Middle_GPIO  GPIOB	 
#define Middle_PIN  GPIO_PIN_12

#define Right1_GPIO GPIOA	 
#define Right1_PIN GPIO_PIN_11

#define Right2_GPIO GPIOB	  
#define Right2_PIN GPIO_PIN_15

#define Left1 HAL_GPIO_ReadPin(Left1_GPIO,Left1_PIN)
#define Left2 HAL_GPIO_ReadPin(Left2_GPIO,Left2_PIN)	 
#define Middle HAL_GPIO_ReadPin(Middle_GPIO,Middle_PIN)
#define Right1 HAL_GPIO_ReadPin(Right1_GPIO,Right1_PIN)
#define Right2 HAL_GPIO_ReadPin(Right2_GPIO,Right2_PIN)

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//	uint8_t RxData[5];
	char RxData[1];
	int ultrasonic = 0;
	int ultrasonic_time;
	unsigned char b[4]={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
	
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//输入
//int fputc(int ch, FILE *f)
//{   
//    while((USART1->SR&0X40)==0); 
//    USART1->DR = (uint8_t) ch;      
//    return ch;
//}
//电机驱动
void Change_Pulse(uint16_t left,uint16_t right)
{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, left);
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_4, right);
	
}
//-----------------------------------------------
void Correct_Left(void)//左转
 {
	Change_Pulse(correct_speed,full_speed);
	HAL_GPIO_WritePin(MOTOR_A_CON1_GPIO, MOTOR_A_CON1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_A_CON2_GPIO, MOTOR_A_CON2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_CON1_GPIO, MOTOR_B_CON1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_CON2_GPIO, MOTOR_B_CON2_PIN, GPIO_PIN_SET);
 }
//------------------------------------------------
 void Correct_Right(void)//右转
 {
	Change_Pulse(full_speed,correct_speed);
	HAL_GPIO_WritePin(MOTOR_A_CON1_GPIO, MOTOR_A_CON1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_A_CON2_GPIO, MOTOR_A_CON2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_CON1_GPIO, MOTOR_B_CON1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_CON2_GPIO, MOTOR_B_CON2_PIN, GPIO_PIN_RESET);
 }
//-----------------------------------------------------
void Straight(void)	//直行
{
	Change_Pulse(full_speed,full_speed);
	HAL_GPIO_WritePin(MOTOR_A_CON1_GPIO, MOTOR_A_CON1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_A_CON2_GPIO, MOTOR_A_CON2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_CON1_GPIO, MOTOR_B_CON1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_CON2_GPIO, MOTOR_B_CON2_PIN, GPIO_PIN_SET); 
}
void Back(void)	//后退
{
	Change_Pulse(full_speed,full_speed);
	HAL_GPIO_WritePin(MOTOR_A_CON1_GPIO, MOTOR_A_CON1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_A_CON2_GPIO, MOTOR_A_CON2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_CON1_GPIO, MOTOR_B_CON1_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_B_CON2_GPIO, MOTOR_B_CON2_PIN, GPIO_PIN_RESET); 
}
void Stop(void)	//停止
{
	Change_Pulse(null_speed,null_speed);
	HAL_GPIO_WritePin(MOTOR_A_CON1_GPIO, MOTOR_A_CON1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_A_CON2_GPIO, MOTOR_A_CON2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_CON1_GPIO, MOTOR_B_CON1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_B_CON2_GPIO, MOTOR_B_CON2_PIN, GPIO_PIN_RESET);
}
//循迹函数
void Xunji(void)
{	
		if(0 == HAL_GPIO_ReadPin(Left2_GPIO,Left2_PIN) && 0 == HAL_GPIO_ReadPin(Right2_GPIO,Right2_PIN))
		{
			Straight();		
		}
		else if(0 == HAL_GPIO_ReadPin(Right2_GPIO,Right2_PIN) || 0 == HAL_GPIO_ReadPin(Right1_GPIO,Right1_PIN))
		{
			Correct_Right();
			HAL_Delay(1000);
		}
		else if(0 == HAL_GPIO_ReadPin(Left2_GPIO,Left2_PIN) || 0 == HAL_GPIO_ReadPin(Left1_GPIO,Left1_PIN))
		{
			Correct_Left();
			HAL_Delay(1000);
		}
		else
		{
			Correct_Left();	
			HAL_Delay(5000);
		}
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);//铚傞福鍣ㄥ厛鍏抽棴
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);	
	

//			HAL_Delay(1000);  				
			
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	HAL_UART_Receive_IT(&huart1,(unsigned char *)RxData, 1);	//从端口接收数据
	unsigned char a[] = "060";
	int ret = strcmp((char *)b,(char *)a);
	  
	if(strncmp(RxData,"z",1) == 0)//循迹
	{
		Xunji();
	}
	else if(strncmp(RxData,"c",1) == 0)//避障
	{
		HAL_UART_Transmit(&huart1,b,3,10);
		if(ret <= 0)
		{		
			Correct_Left();
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);//灯亮		
		}
		else
		{		
			Straight();				
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);	//灯灭		
		}
	}	
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)//控制函数，小车行进
{	
	HAL_UART_Transmit(&huart1,(unsigned char *)RxData,1,10);
	if(strncmp(RxData,"g",1) == 0)
	{			
		Straight();
	}
	else if(strncmp(RxData,"l",1) == 0)
	{     			
		Correct_Left();
	}
	else if(strncmp(RxData,"r",1) == 0)
	{      		
		Correct_Right();
	}
	else if(strncmp(RxData,"b",1) == 0)
	{     	
		Back();
	}
	else if(strncmp(RxData,"s",1) == 0)
	{   		
		Stop();
	}		
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim==&htim6)
    {
        HAL_TIM_Base_Stop_IT(&htim6);
        ultrasonic=0;
        HAL_GPIO_WritePin(Trig_GPIO, Trig_PIN, GPIO_PIN_SET);
		HAL_TIM_Base_Start_IT(&htim7);
    }
    else if(htim==&htim7)
    {
        HAL_GPIO_WritePin(Trig_GPIO,Trig_PIN, GPIO_PIN_RESET);
        ultrasonic=1;
        HAL_TIM_Base_Stop_IT(&htim7);
        HAL_TIM_Base_Start_IT(&htim6);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//中断回调函数
{
	HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_6);	 
    if(GPIO_Pin==GPIO_PIN_2)
    {     
		if(HAL_GPIO_ReadPin(Echo_GPIO, Echo_PIN))
        {	
            ultrasonic_time=__HAL_TIM_GET_COUNTER(&htim6);
        }
        else if(ultrasonic==1)
        {
            ultrasonic=0;
            ultrasonic_time=__HAL_TIM_GET_COUNTER(&htim6)-ultrasonic_time;
            ultrasonic_time=ultrasonic_time*170/100;		
            ultrasonic_time=ultrasonic_time%1000;//?cm???,????
            b[0]=ultrasonic_time/100+'0';
            ultrasonic_time=ultrasonic_time%100;
            b[1]=ultrasonic_time/10+'0';
            b[2]=ultrasonic_time%10+'0';
            ultrasonic_time=0;  
			//HAL_UART_Transmit(&huart1,b,3,10);			
        }       
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
