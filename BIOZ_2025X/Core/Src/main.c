/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <string.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void delay_us(uint32_t us);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define AD9851_RESET_PORT RESET_GPIO_Port
#define AD9851_RESET_PIN RESET_Pin

#define AD9851_DATA_PORT DATA_GPIO_Port
#define AD9851_DATA_PIN DATA_Pin

#define AD9851_FQUD_PORT FQ_UD_GPIO_Port
#define AD9851_FQUD_PIN FQ_UD_Pin

#define AD9851_WCLK_PORT W_CLK_GPIO_Port
#define AD9851_WCLK_PIN W_CLK_Pin




#define pulseHigh(pin)  { HAL_GPIO_WritePin(pin##_GPIO_Port, pin##_Pin, GPIO_PIN_SET); \
                          HAL_GPIO_WritePin(pin##_GPIO_Port, pin##_Pin, GPIO_PIN_RESET); }

//Log Sweep Parameter
//Freq Sweeping Parameters (1 KHz to 9 KHz)
long unsigned int start_freq_1=1000;   //start frequency (Hz)
long unsigned int stop_freq_1=10000;  //stop frequency (Hz) (N.B.: Don't Sweep over 60 MHz and at 25 MHz)
long unsigned int freq_step_1=1000;   //frequency step size (Hz)


//Freq Sweeping Parameters (10 KHz to 100 KHz)
long unsigned int start_freq_2=10000;   //start frequency (Hz)
long unsigned int stop_freq_2= 100000;  //stop frequency (Hz) (N.B.: Don't Sweep over 60 MHz and at 25 MHz)
long unsigned int freq_step_2= 10000;   //frequency step size (Hz)


//Freq Sweeping Parameters (100 KHz to 1000 KHz)
long unsigned int start_freq_3=	100000;   //start frequency (Hz)
long unsigned int stop_freq_3=	1000000;  //stop frequency (Hz) (N.B.: Don't Sweep over 60 MHz and at 25 MHz)
long unsigned int freq_step_3=	100000;   //frequency step size (Hz)


////Linear Sweep Parameter (10 KHz to 1 MHz)
//long unsigned int start_freq=10e3;   //start frequency (Hz)
//long unsigned int stop_freq=1000e3;  //stop frequency (Hz) (N.B.: Don't Sweep over 60 MHz and at 25 MHz)
//long unsigned int freq_step=10e3;   //frequency step size (Hz)


int arr[] = {1000, 1072, 1150, 1233, 1322, 1417, 1520, 1630, 1748, 1874,
			 2009, 2154, 2310, 2477, 2656, 2848, 3054, 3275, 3511, 3765,
			 4037, 4329, 4642, 4977, 5337, 5722, 6136, 6579, 7055, 7565,
			 8111, 8697, 9326, 10000, 10723, 11498, 12328, 13219, 14175, 15199,
			 16298, 17475, 18738, 20092, 21544, 23101, 24771, 26561, 28480, 30539,
			 32745, 35112, 37649, 40370, 43288, 46416, 49770, 53367, 57224, 61359,
			 65793, 70548, 75646, 81113, 86975, 93260, 100000, 107227, 114976, 123285,
			 132194, 141747, 151991, 162975, 174753, 187382, 200923, 215443, 231013, 247708,
			 265609, 284804, 305386, 327455, 351119, 376494, 403702, 432876, 464159, 497702,
			 533670, 572237, 613591, 657933, 705480, 756463, 811131, 869749, 932603, 1000000};



long unsigned int freq; // freq variable for sweep
uint8_t data_ready_flag = 0;

#define ADC_BUFF_LEN 1024
uint16_t adc_buff[ADC_BUFF_LEN];

void tfr_byte(uint8_t data);
void sendFrequency(double frequency);

// transfer a byte a bit at a time LSB first to DATA
void tfr_byte(uint8_t data)
{
  for (int i=0; i<8; i++, data>>=1) {
    HAL_GPIO_WritePin(DATA_GPIO_Port, DATA_Pin, data & 0x01);
    pulseHigh(W_CLK);
  }
}

// frequency of signwave (datasheet page 12) will be:
//   <sys clock> * <frequency tuning word> / 2^32
void sendFrequency(double frequency) {
//  int32_t freq = frequency * 4294967296.0 / 180000 000;
  int32_t freq = frequency * 4294967296.0 / 180000000;
  for (int b=0; b<4; b++, freq>>=8) {
    tfr_byte(freq & 0xFF);
  }
  tfr_byte(0x81);  // = 0x81: PLL ON + LSB-first mode
  pulseHigh(FQ_UD);
}



void delay_us(uint32_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset counter
    while (__HAL_TIM_GET_COUNTER(&htim2) < us); // Wait until counter reaches 'us'
}



void setup_mux(void){

	//	  T26 with T32
	//	  T23 with T29

	// T23, LC+_1 X1Y1
	HAL_GPIO_WritePin(GPIOB, mux_i_a_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, mux_i_b_Pin, GPIO_PIN_RESET);

	// T26-HV+1,  T32-LV-1   A-HIGH, B-LOW,  X1Y1
	HAL_GPIO_WritePin(mux_v_a_GPIO_Port, mux_v_a_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(mux_v_b_GPIO_Port, mux_v_b_Pin, GPIO_PIN_RESET);

	// write code to

}



void setup_mux_x(int val){


	// switch mux to select between three steps
	/**
	 * Switch one. LowCurrent1 and HighCurrent 1 and HighVoltage1-LowVoltage1
	 * Each step will choose which connector to select from
	 */
	switch(val){
	case 1:
		// T23-T29, T26-T32
		// LC+_1 - LC-_1,	 HV+_1 - LV-_1
		HAL_GPIO_WritePin(GPIOB, mux_i_a_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, mux_i_b_Pin, GPIO_PIN_RESET);

		// T23-T29, T26-T32
		// LC+_1 - LC-_1,	 HV+_1 - LV-_1
		HAL_GPIO_WritePin(mux_v_a_GPIO_Port, mux_v_a_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mux_v_b_GPIO_Port, mux_v_b_Pin, GPIO_PIN_RESET);
		break;
	case 2:
		// T24-T30, T27-T33
		// LC+_1 - LC-_1,	 HV+_1 - LV-_1
		HAL_GPIO_WritePin(GPIOB, mux_i_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, mux_i_b_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(mux_v_a_GPIO_Port, mux_v_a_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(mux_v_b_GPIO_Port, mux_v_b_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(GPIOB, mux_i_a_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, mux_i_b_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(mux_v_a_GPIO_Port, mux_v_a_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(mux_v_b_GPIO_Port, mux_v_b_Pin, GPIO_PIN_SET);
		break;
	default:
		// do nothing
		break;
	}

}

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	setup_mux();


	pulseHigh(DATA);
	pulseHigh(RESET);
	pulseHigh(W_CLK);
	pulseHigh(FQ_UD);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buff, ADC_BUFF_LEN);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {



//	 put your main code here, to run repeatedly:
//	(1KHz to 9KHz)
	for(freq=start_freq_1; freq<=stop_freq_1; freq += freq_step_1)
	{
		sendFrequency(3000);
		HAL_Delay(30);// 100 mS delay
	}

//	for(int i=0; i<100; i++){
//		sendFrequency(arr[i]);
//		HAL_Delay(90);// 100
//	}

	sendFrequency(1000000);
	HAL_Delay(90);// 100


	//Freq Sweeping Parameters (10 KHz to 100 KHz)
	for(freq=start_freq_2; freq<=stop_freq_2; freq += freq_step_2)
	{
		sendFrequency(freq);
		HAL_Delay(100);// 100 mS delay
	}


//	Freq Sweeping Parameters (10 KHz to 100 KHz)
	for(freq=start_freq_3; freq<=stop_freq_3; freq += freq_step_3)
	{
		sendFrequency(freq);
		HAL_Delay(100);// 100 mS delay
	}


/*

	if(data_ready_flag){
		  for(int i = 0; i < ADC_BUFF_LEN; i++){
			  char buffer[4];
			  sprintf(buffer, "%u\r\n", adc_buff[i]);
			  HAL_UART_Transmit(&huart3, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
		  }
		  data_ready_flag = 0;
	}

	*/


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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 68-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, mux_i_a_Pin|mux_i_b_Pin|FQ_UD_Pin|W_CLK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RESET_Pin|DATA_Pin|mux_v_a_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(mux_v_b_GPIO_Port, mux_v_b_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : mux_i_a_Pin mux_i_b_Pin */
  GPIO_InitStruct.Pin = mux_i_a_Pin|mux_i_b_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RESET_Pin DATA_Pin */
  GPIO_InitStruct.Pin = RESET_Pin|DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : mux_v_a_Pin */
  GPIO_InitStruct.Pin = mux_v_a_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(mux_v_a_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : mux_v_b_Pin */
  GPIO_InitStruct.Pin = mux_v_b_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(mux_v_b_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FQ_UD_Pin */
  GPIO_InitStruct.Pin = FQ_UD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FQ_UD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : W_CLK_Pin */
  GPIO_InitStruct.Pin = W_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(W_CLK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	  data_ready_flag = 1;
}

// write functions to swich mux



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
