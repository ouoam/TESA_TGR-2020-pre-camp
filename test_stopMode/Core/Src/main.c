/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint32_t tmpCC4[2] = {0, 0};
uint32_t uwLsiFreq = 0;

uint32_t uwCaptureNumber = 0;
uint32_t uwPeriodValue = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM21_Init(void);
/* USER CODE BEGIN PFP */

static uint32_t GetLSIFrequency(void);

void _stm32l_disableGpios() {

   // GPIO_A is preserved to keep output status unchanged and have
   // Interrupt working for waking Up.


   GPIO_InitTypeDef GPIO_InitStruct = {0};
   GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
   GPIO_InitStruct.Pull = GPIO_NOPULL;

   /*Configure GPIO pins : PA15_RESERVED_Pin PA12_RESERVED_Pin PA1_RESERVED_Pin */
     GPIO_InitStruct.Pin = PA15_RESERVED_Pin|PA12_RESERVED_Pin|PA1_RESERVED_Pin;
     HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

     /*Configure GPIO pins : PB4_RESERVED_Pin PB1_RESERVED_Pin PB0_RESERVED_Pin */
     GPIO_InitStruct.Pin = PB4_RESERVED_Pin|PB1_RESERVED_Pin|PB0_RESERVED_Pin;
     HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

     /*Configure GPIO pin : PC13_RESERVED_Pin */
     GPIO_InitStruct.Pin = PC13_RESERVED_Pin;
     HAL_GPIO_Init(PC13_RESERVED_GPIO_Port, &GPIO_InitStruct);

     /*Configure GPIO pins : PC1_RESERVED_Pin PC0_RESERVED_Pin PC2_RESERVED_Pin */
     GPIO_InitStruct.Pin = PC1_RESERVED_Pin|PC0_RESERVED_Pin|PC2_RESERVED_Pin;
     HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

   __HAL_RCC_GPIOA_CLK_DISABLE();
   	  __HAL_RCC_GPIOB_CLK_DISABLE();
   	  __HAL_RCC_GPIOC_CLK_DISABLE();
   	  __HAL_RCC_GPIOH_CLK_DISABLE();

}

void stm32l_lowPowerSetup () {

	__HAL_RCC_PWR_CLK_ENABLE(); // Enable Power Control clock
	HAL_PWREx_EnableUltraLowPower(); // Ultra low power mode
	HAL_PWREx_EnableFastWakeUp(); // Fast wake-up for ultra low power mode

	// Disable Unused Gpios
	_stm32l_disableGpios(); // Disable GPIOs based on configuration


	// Configure LPUART for Wake-up
	// make sure that no UART transfer is on-going
	while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_BUSY) == SET);
	// make sure that UART is ready to receive
	while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_REACK) == RESET);

	UART_WakeUpTypeDef wakeup;
	wakeup.WakeUpEvent=UART_WAKEUP_ON_STARTBIT; // UART_WAKEUP_ON_READDATA_NONEMPTY
	HAL_UARTEx_StopModeWakeUpSourceConfig(&huart2,wakeup);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_WUF);
	HAL_UARTEx_EnableStopMode(&huart2);

	// GPIO Wake up is configured by CubeMx config and keeping the GPIO activated

	HAL_SuspendTick();
	// Switch to STOPMode
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

	//uint32_t _time = 10 * 60 * 1.25;
	//HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, _time, RTC_WAKEUPCLOCK_CK_SPRE_16BITS);
}

void stm32l_lowPowerResume () {
	HAL_ResumeTick();
	// Reinit clocks
	SystemClock_Config();
	// Deactivate RTC wakeUp
	//HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
	// Reinit GPIOs
	MX_GPIO_Init();
	//MX_RTC_Init();

	HAL_UARTEx_DisableStopMode(&huart2);

	// Configure RTC to wake up after 500ms


	// Reinit LPUART
	  MX_SPI1_Init();
	  MX_USART2_UART_Init();
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t wakeup = 0;
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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_TIM21_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_UART_Transmit(&huart2, (uint8_t *)"H", 1, 10);
	  HAL_UART_Transmit(&huart2, &wakeup, 1, 10);
	  wakeup = 0;
	  // HAL_Delay(1000);
	  stm32l_lowPowerSetup();
	      // sleeping
	      stm32l_lowPowerResume();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */
	uwLsiFreq = GetLSIFrequency();
	uint8_t SetAsynch = 0x7F;
	uint16_t bestAccurate = 0xFFFF;
	for (uint8_t i = 0 ; i < 0x7F; i++) {
		uint32_t SetSynch = (uwLsiFreq / (i + 1)) - 1;
		if (SetSynch > 0x1FFF) continue;
		int32_t abs = ((SetSynch + 1) * (i + 1)) - uwLsiFreq;
		if (abs < 0) abs = -abs;
		if (abs <= bestAccurate) {
			SetAsynch = i;
			bestAccurate = abs;
		}
	}
  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = SetAsynch;
  hrtc.Init.SynchPrediv = (uwLsiFreq/(SetAsynch + 1)) - 1;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the WakeUp 
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 5, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 0;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 0xFFFF;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim21, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIMEx_RemapConfig(&htim21, TIM21_TI1_LSI) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */
  HAL_TIM_IC_DeInit(&htim21);
  /* USER CODE END TIM21_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, PA15_RESERVED_Pin|PA12_RESERVED_Pin|PA1_RESERVED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, PC1_RESERVED_Pin|PC0_RESERVED_Pin|PC2_RESERVED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA15_RESERVED_Pin PA12_RESERVED_Pin PA1_RESERVED_Pin */
  GPIO_InitStruct.Pin = PA15_RESERVED_Pin|PA12_RESERVED_Pin|PA1_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4_RESERVED_Pin PB1_RESERVED_Pin PB0_RESERVED_Pin */
  GPIO_InitStruct.Pin = PB4_RESERVED_Pin|PB1_RESERVED_Pin|PB0_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13_RESERVED_Pin */
  GPIO_InitStruct.Pin = PC13_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PC13_RESERVED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1_RESERVED_Pin PC0_RESERVED_Pin PC2_RESERVED_Pin */
  GPIO_InitStruct.Pin = PC1_RESERVED_Pin|PC0_RESERVED_Pin|PC2_RESERVED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Configures TIM21 to measure the LSI oscillator frequency.
  * @param  None
  * @retval LSI Frequency
  */
static uint32_t GetLSIFrequency(void)
{
	MX_TIM21_Init();
	HAL_TIM_IC_Init(&htim21);

  /* Start the TIM Input Capture measurement in interrupt mode */
  if(HAL_TIM_IC_Start_IT(&htim21, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Wait until the TIM21 get 2 LSI edges */
  while(uwCaptureNumber != 2)
  {
  }

  /* Disable TIM21 CC1 Interrupt Request */
  HAL_TIM_IC_Stop_IT(&htim21, TIM_CHANNEL_1);

  /* Deinitialize the TIM21 peripheral registers to their default reset values */
  HAL_TIM_IC_DeInit(&htim21);

  return uwLsiFreq;
}

/**
  * @brief  Input Capture callback in non blocking mode
  * @param  htim : TIM IC handle
  * @retval None
*/
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Get the Input Capture value */
  tmpCC4[uwCaptureNumber++] = HAL_TIM_ReadCapturedValue(&htim21, TIM_CHANNEL_1);

  if (uwCaptureNumber >= 2)
  {
    if ( tmpCC4[0] > tmpCC4[1] )
    {
      /* Compute the period length */
      uwPeriodValue = 0xFFFFFFFF - tmpCC4[0] + tmpCC4[1] + 1;
    }
    else
    {
      /* Compute the period length */
      uwPeriodValue = tmpCC4[1] - tmpCC4[0];
    }
    /* Frequency computation */
    uwLsiFreq = (uint32_t) SystemCoreClock / uwPeriodValue;
  }
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  /* Clear Wake Up Flag */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
  wakeup = 1;
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
