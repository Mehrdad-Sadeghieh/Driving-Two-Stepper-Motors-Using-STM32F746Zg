/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"

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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

int STATE_IND = 0 ,STOP_IND = 400 ,STATE_FILE_PRO = 0 ,Rx_complate = 1 ,once = 1 ,i = 0 ,j = 0;
char file[1] = {5}  ,file_copy[300000] ,BUFF[16];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

void delay_us(uint16_t us);

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
  MX_TIM1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	


  HAL_SuspendTick();
  HAL_TIM_Base_Start(&htim1);
  memset(file_copy ,'5' ,sizeof(file_copy));
	
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	       
	   if (once == 1)
	   {
   	    for (i = 0 ;i <= 299999 ;i++)
        {
	          HAL_UART_Receive(&huart4 ,(unsigned char *)file ,1,100000);
		        file_copy[i] = file[0];
        }	   
	      once = 0;
        STATE_IND = 0;	
        GPIOB->BSRR = LED_BLUE_Pin;			
  	 }
		
	  while (STATE_FILE_PRO == 1)                                                        // start after process file
	  {
	     GPIOB->BSRR = LED_RED_Pin;
       switch (file_copy[STATE_IND])
       {
         case 1:
		       GPIOA->BSRR = GPIO_PIN_5;                                           // 5
	         GPIOA->BSRR = GPIO_PIN_7;								 													 // 7
		       GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		       GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		       delay_us(500);
	         GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
	         GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
           delay_us(500);			  
		       STATE_IND += 4;
      	   break;
      	 case 2:
		       GPIOA->BSRR = GPIO_PIN_5;                                           // 5
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;													 // 7
		       GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		       GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		       delay_us(500);
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
	     	   GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
	         delay_us(500);
	         STATE_IND += 4;
           break;
    	   case 3:
		       GPIOA->BSRR = GPIO_PIN_5;                                           // 5
	         GPIOA->BSRR = GPIO_PIN_7;																					 // 7
	         GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		       GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		       delay_us(500);
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		       delay_us(500);
		       STATE_IND += 4;
		       break;
	       case 4:
	         GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16;                           // 5
	         GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		       GPIOA->BSRR = GPIO_PIN_4;																					 // 4
	         GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		       delay_us(500);
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		       delay_us(500);
		       STATE_IND += 4;
		       break;
	       case 5:
	         GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16;                           // 5
	         GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;													 // 7
	         GPIOA->BSRR = GPIO_PIN_4;																					 // 4
	      	 GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		       delay_us(500);
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		       delay_us(500);
		       STATE_IND += 4;
		       break;
	       case 6:
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_5 << 16;                           // 5
		       GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		       GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		       GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		       delay_us(500);
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_4 << 16;
		       delay_us(500);
		       STATE_IND += 4;
		       break;
	    	 case 7:
		       GPIOA->BSRR = GPIO_PIN_5;                                           // 5
		       GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		       GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		       GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		       delay_us(500);
		       GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		       delay_us(500);
		       STATE_IND += 4;
	   	     break;
	      case 8:
		      GPIOA->BSRR = GPIO_PIN_5;                                           // 5
		      GPIOA->BSRR = (uint32_t)GPIO_PIN_7 << 16;				  								 // 7
		      GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		      GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		      delay_us(500);
		      GPIOA->BSRR = (uint32_t)GPIO_PIN_6 << 16;
		      delay_us(500);
		      STATE_IND += 4;
		      break;
	      case 9:
		      GPIOA->BSRR = GPIO_PIN_5;                                           // 5
		      GPIOA->BSRR = GPIO_PIN_7;																					 // 7
		      GPIOA->BSRR = GPIO_PIN_4;																					 // 4
		      GPIOA->BSRR = GPIO_PIN_6;																					 // 6
		      delay_us(1000);
		      STATE_IND += 4;
		      break;
       }
	     if (STATE_IND >= STOP_IND)                                                      // end of file and stop running motor
	     {
		      STATE_FILE_PRO = 0;
		      STATE_IND = 0;
	     }
	  }

    if (Rx_complate == 1)                                                           // receive hole file and start moving motor
  	{
	     for (STATE_IND = 40 ;STATE_IND <= 299999 ;STATE_IND++)
       {
		       if (file_copy[STATE_IND] == 50)
		       {
		           STOP_IND = STATE_IND;
		           break;
	         }
       }
       j = STOP_IND/4;
			 STATE_IND = 0;
	     for (i = 0 ;i <= j ;i++)
	     {
	         if (file_copy[STATE_IND] == 49 && file_copy[STATE_IND + 1] == 48 && file_copy[STATE_IND + 2] == 49 && file_copy[STATE_IND +3] == 48)
	         {
              file_copy[STATE_IND] = 1;
		          STATE_IND += 4;
           }
	         if (file_copy[STATE_IND] == 48 && file_copy[STATE_IND + 1] == 49 && file_copy[STATE_IND + 2] == 48 && file_copy[STATE_IND +3] == 49)
	         {
		          file_copy[STATE_IND] = 5;
		          STATE_IND += 4;
           }
	         if (file_copy[STATE_IND] == 48 && file_copy[STATE_IND + 1] == 48 && file_copy[STATE_IND + 2] == 48 && file_copy[STATE_IND +3] == 48)
	         {
	     	      file_copy[STATE_IND] = 9;
		          STATE_IND += 4;
	         }
	         if (file_copy[STATE_IND] == 49 && file_copy[STATE_IND + 1] == 48 && file_copy[STATE_IND + 2] == 48 && file_copy[STATE_IND +3] == 49)
	         {
	    	      file_copy[STATE_IND] = 2;
	    	      STATE_IND += 4;
	         }
	         if (file_copy[STATE_IND] == 49 && file_copy[STATE_IND + 1] == 48 && file_copy[STATE_IND + 2] == 48 && file_copy[STATE_IND +3] == 48)
	         {
	    	      file_copy[STATE_IND] = 3;
	    	      STATE_IND += 4;
	         }
	         if (file_copy[STATE_IND] == 48 && file_copy[STATE_IND + 1] == 49 && file_copy[STATE_IND + 2] == 49 && file_copy[STATE_IND +3] == 48)
	         {
	    	      file_copy[STATE_IND] = 4;
	    	      STATE_IND += 4;
	         }
	        if (file_copy[STATE_IND] == 48 && file_copy[STATE_IND + 1] == 49 && file_copy[STATE_IND + 2] == 48 && file_copy[STATE_IND +3] == 48)
	        {
	    	     file_copy[STATE_IND] = 6;
	    	     STATE_IND += 4;
	        }
	        if (file_copy[STATE_IND] == 48 && file_copy[STATE_IND + 1] == 48 && file_copy[STATE_IND + 2] == 49 && file_copy[STATE_IND +3] == 48)
	        {
	    	     file_copy[STATE_IND] = 7;
	    	     STATE_IND += 4;
	        }
	        if (file_copy[STATE_IND] == 48 && file_copy[STATE_IND + 1] == 48 && file_copy[STATE_IND + 2] == 48 && file_copy[STATE_IND +3] == 49)
	        {
	    	     file_copy[STATE_IND] = 8;
	    	     STATE_IND += 4;
	        }
       }
	     GPIOB->BSRR = LED_GREEN_Pin;
	     STATE_IND = 0;
	     STATE_FILE_PRO = 1;
	     Rx_complate = 0;
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 215;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LCD_RS_Pin|LCD_RW_Pin|LCD_EN_Pin|LCD_D0_Pin 
                          |LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin|LCD_D4_Pin 
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PUL_1_Pin|PUL_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DR_1_Pin|DR_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin|LED_BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_RW_Pin LCD_EN_Pin LCD_D0_Pin 
                           LCD_D1_Pin LCD_D2_Pin LCD_D3_Pin LCD_D4_Pin 
                           LCD_D5_Pin LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_RW_Pin|LCD_EN_Pin|LCD_D0_Pin 
                          |LCD_D1_Pin|LCD_D2_Pin|LCD_D3_Pin|LCD_D4_Pin 
                          |LCD_D5_Pin|LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL_1_Pin DR_1_Pin PUL_2_Pin DR_2_Pin */
  GPIO_InitStruct.Pin = PUL_1_Pin|DR_1_Pin|PUL_2_Pin|DR_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin LED_BLUE_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin|LED_BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1 ,0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < us)
	{
		
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
