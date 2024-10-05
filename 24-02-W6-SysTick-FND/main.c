/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../BMEAL/SEVEN_SEGMENTS/SEVEN_SEGMENTS.h"
#include "Serial_printf.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t SysTicks = 0;


int _Dir, SW1Flag, SW2Flag, StartFlag, icnt;
uint16_t fndCount;
uint8_t data[4] ={0,0,0,0}, _digit=0;
float myDuty;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void Seg_Reset( void );
void Seg_Number( uint16_t val, uint8_t dp);

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  if( HAL_TIM_Base_Start_IT( &htim3) != HAL_OK){
  	  Error_Handler();
    }

  SEVEN_SEG_Init(0);
  SEVEN_SEG_Enable(0);
  SEVEN_SEG_Write(0, 1234);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // FND의 LED가 정상동작 하는지 확인하기 위한 코드...
//    HAL_GPIO_WritePin(GPIOA, FND_SEL1_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOC, FND_SEL2_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin|FND5_Pin|FND6_Pin|FND7_Pin, GPIO_PIN_SET);

  StartFlag=0;
  icnt=0;
  myDuty = 0.0;
  fndCount = 0;


  while (1)
  {
	  if( StartFlag && fndCount != icnt){
		  printf("Hello World!......(  %3d) / %2d \n ", icnt, StartFlag);
		  SEVEN_SEG_Write(0, fndCount);
		  fndCount = icnt;
	  }
//	  SEVEN_SEG_Write(0, icnt);

	  //////////////////////////////// Manual FND manipulation.......
////	  Seg_Reset();
////	  HAL_GPIO_WritePin(GPIOA, FND_SEL0_Pin, GPIO_PIN_SET);
////	  HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin|FND5_Pin,GPIO_PIN_SET);  // 0
//
//	  HAL_GPIO_WritePin(GPIOA, FND_SEL0_Pin, GPIO_PIN_SET);
////	  Seg_Number(8);
//	  if( _digit == 0)
//		  Seg_Number(data[0],1);
//	  else
//		  Seg_Number(data[0],0);
//	  HAL_Delay(5);
//	  HAL_GPIO_WritePin(GPIOA, FND_SEL0_Pin, GPIO_PIN_RESET);
//
////	  Seg_Reset();
//	  HAL_GPIO_WritePin(GPIOA, FND_SEL1_Pin, GPIO_PIN_SET);
////	  HAL_GPIO_WritePin(GPIOB, FND1_Pin|FND2_Pin,GPIO_PIN_SET);  // 1
////	  Seg_Number(9);
//	  if( _digit == 1)
//		  Seg_Number(data[1],1);
//	  else
//	  	  Seg_Number(data[1],0);
//	  HAL_Delay(5);
//	  HAL_GPIO_WritePin(GPIOA, FND_SEL1_Pin, GPIO_PIN_RESET);
//
////	  Seg_Reset();
//	  HAL_GPIO_WritePin(GPIOC, FND_SEL2_Pin, GPIO_PIN_SET);
////	  HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND3_Pin|FND4_Pin|FND6_Pin,GPIO_PIN_SET);  // 2
////	  Seg_Number(0);
//	  if( _digit == 2)
//		  Seg_Number(data[2],1);
//	  else
//	  	  Seg_Number(data[2],0);
//	  HAL_Delay(5);
//	  HAL_GPIO_WritePin(GPIOC, FND_SEL2_Pin, GPIO_PIN_RESET);
//
////	  Seg_Reset();
//	  HAL_GPIO_WritePin(GPIOC, FND_SEL3_Pin, GPIO_PIN_SET);
////	  HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin|FND6_Pin,GPIO_PIN_SET);  // 3
////	  Seg_Number(7);
//	  if( _digit == 3)
//		  Seg_Number(data[3],1);
//	  else
//	  	  Seg_Number(data[3],0);
//  	  HAL_Delay(5);
//  	  HAL_GPIO_WritePin(GPIOC, FND_SEL3_Pin, GPIO_PIN_RESET);
//
////  	  HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin
////  			  |FND4_Pin|FND5_Pin|FND6_Pin|FND7_Pin, GPIO_PIN_RESET);
//
//  	  if( SW1Flag ){
//  		  SW1Flag=0;
//  		  data[_digit]++;
//  		  data[_digit] %= 10;
//  	  }
//  	  if(SW2Flag){
//  		  SW2Flag=0;
//  		  _digit++;
//  		  _digit %= 4;
//
//  		  myDuty += 10.0;
//  		  myDuty = (float)((int)myDuty%100);
////  		  adjust_PWM_dutyCycle(&htim2, TIM_CHANNEL_1, myDuty );
//
//  	  }
  	//////////////////////////////// Manual FND manipulation.......



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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8339;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FND_SEL3_Pin|FND_SEL2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|FND_SEL0_Pin|FND_SEL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin
                          |FND4_Pin|FND5_Pin|FND6_Pin|FND7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FND_SEL3_Pin FND_SEL2_Pin */
  GPIO_InitStruct.Pin = FND_SEL3_Pin|FND_SEL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin FND_SEL0_Pin FND_SEL1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|FND_SEL0_Pin|FND_SEL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FND0_Pin FND1_Pin FND2_Pin FND3_Pin
                           FND4_Pin FND5_Pin FND6_Pin FND7_Pin */
  GPIO_InitStruct.Pin = FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin
                          |FND4_Pin|FND5_Pin|FND6_Pin|FND7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void Seg_Reset( void ){
	HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin
			  |FND4_Pin|FND5_Pin|FND6_Pin|FND7_Pin, GPIO_PIN_RESET);
}

void Seg_Number( uint16_t val, uint8_t dp){

	uint16_t _bits;

	_bits = FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin|FND5_Pin;  // "0"
	// Clear All segment port bits...
	HAL_GPIO_WritePin(GPIOB, FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin
				  |FND4_Pin|FND5_Pin|FND6_Pin|FND7_Pin, GPIO_PIN_RESET);

	switch(val%10){
		case 1:
			_bits = FND1_Pin|FND2_Pin;
			break;
		case 2:
			_bits = FND0_Pin|FND1_Pin|FND3_Pin|FND4_Pin|FND6_Pin;
			break;
		case 3:
			_bits = FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin|FND6_Pin;
			break;
		case 4:
			_bits = FND1_Pin|FND2_Pin|FND5_Pin|FND6_Pin;
			break;
		case 5:
			_bits = FND0_Pin|FND2_Pin|FND3_Pin|FND5_Pin|FND6_Pin;
			break;
		case 6:
			_bits = FND2_Pin|FND3_Pin|FND4_Pin|FND5_Pin|FND6_Pin;
			break;
		case 7:
			_bits = FND0_Pin|FND1_Pin|FND2_Pin;
			break;
		case 8:
			_bits = FND0_Pin|FND1_Pin|FND2_Pin|FND3_Pin|FND4_Pin|FND5_Pin|FND6_Pin;
			break;
		case 9:
			_bits = FND0_Pin|FND1_Pin|FND2_Pin|FND5_Pin|FND6_Pin;
			break;
	}
	if(dp)
		_bits |= FND7_Pin;
	HAL_GPIO_WritePin(GPIOB, _bits, GPIO_PIN_SET);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){

//	icnt++;
	if(GPIO_Pin == SW1_Pin){
		fndCount--;
		SW1Flag = 1;  SW2Flag = 0;
		if( HAL_GPIO_ReadPin(SW2_GPIO_Port,SW2_Pin)== GPIO_PIN_RESET )
			StartFlag = 1 - StartFlag;
	}else if(GPIO_Pin == SW2_Pin){
		fndCount++;
		SW1Flag = 0;	SW2Flag = 1;
		if( HAL_GPIO_ReadPin(SW1_GPIO_Port,SW1_Pin)== GPIO_PIN_RESET )
			StartFlag = 1 - StartFlag;
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
	icnt++;

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_TIM_PeriodElapsedCallback could be implemented in the user file
   */
}

void SysTick_CallBack(void)
{
    SysTicks++;
    if(!(SysTicks%5))  // Each 5 msec
    {
    	SEVEN_SEG_Main();
    	SysTicks = 0;
    }
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