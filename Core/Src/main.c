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
#include <stdio.h>
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
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t encoder_1_key_flag = 0;
uint32_t encoder_1_key_time = 0;
uint8_t encoder_1_key_short_state = 0;
uint8_t encoder_1_key_long_state = 0;

uint8_t encoder_2_key_flag = 0;
uint32_t encoder_2_key_time = 0;
uint8_t encoder_2_key_encoder_1_key_short_state = 0;
uint8_t encoder_2_key_encoder_1_key_long_state = 0;

uint16_t encoders_start_counter = 32767;
uint16_t encoder_1_prev_counter = 32767;
uint16_t encoder_2_encoder_1_prev_counter = 32767;
uint16_t encoder_1_cur_counter, encoder_2_encoder_1_cur_counter;

uint8_t transmit_data[20];
uint32_t tx_time;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */
	__HAL_TIM_SET_COUNTER(&htim3, encoders_start_counter);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim4, encoders_start_counter);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	uint8_t message[50] = { '\0' };
	transmit_data[4] = 127;
	transmit_data[5] = 127;
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		//��������� �������� 1
		encoder_1_cur_counter = __HAL_TIM_GET_COUNTER(&htim3);
		if (encoder_1_cur_counter != encoder_1_prev_counter) {
			if (encoder_1_cur_counter > encoder_1_prev_counter) {
				if (transmit_data[4] > 251) {
					transmit_data[4] = 255;
				} else {
					transmit_data[4] += 2;
				}
			} else {
				if (transmit_data[4] < 4) {
					transmit_data[4] = 0;
				} else {
					transmit_data[4] -= 2;
				}
			}
			encoder_1_prev_counter = encoder_1_cur_counter;
		}
		//��������� �������� 2
		encoder_2_encoder_1_cur_counter = __HAL_TIM_GET_COUNTER(&htim4);
		if (encoder_2_encoder_1_cur_counter != encoder_2_encoder_1_prev_counter) {
			if (encoder_2_encoder_1_cur_counter > encoder_2_encoder_1_prev_counter) {
				if (transmit_data[5] > 251) {
					transmit_data[5] = 255;
				} else {
					transmit_data[5] += 2;
				}
			} else {
				if (transmit_data[5] < 4) {
					transmit_data[5] = 0;
				} else {
					transmit_data[5] -= 2;
				}
			}
			encoder_2_encoder_1_prev_counter = encoder_2_encoder_1_cur_counter;
		}

		//��������� ������ �������� 1
		if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin)) {
			encoder_1_key_flag = 1;
		}
		if (encoder_1_key_flag) {
			uint32_t ms = HAL_GetTick();
			uint8_t key1_state = HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin);
			if (key1_state == 0 && !encoder_1_key_short_state && (ms - encoder_1_key_time) > 50) {
				encoder_1_key_short_state = 1;
				encoder_1_key_long_state = 0;
				encoder_1_key_time = ms;
			} else if (key1_state == 0 && !encoder_1_key_long_state
					&& (ms - encoder_1_key_time) > 2000) {
				encoder_1_key_long_state = 1;

			} else if (key1_state == 1 && encoder_1_key_short_state
					&& (ms - encoder_1_key_time) > 50) {
				encoder_1_key_short_state = 0;
				encoder_1_key_time = ms;
				if (!encoder_1_key_long_state) {

					transmit_data[4] = 127;
				}
			}
		}

		//��������� ������ �������� 2
		if (HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin)) {
			encoder_2_key_flag = 1;
		}
		if (encoder_2_key_flag) {
			uint32_t ms_2 = HAL_GetTick();
			uint8_t key2_state = HAL_GPIO_ReadPin(KEY_2_GPIO_Port, KEY_2_Pin);
			if (key2_state == 0 && !encoder_2_key_encoder_1_key_short_state && (ms_2 - encoder_2_key_time) > 50) {
				encoder_2_key_encoder_1_key_short_state = 1;
				encoder_2_key_encoder_1_key_long_state = 0;
				encoder_2_key_time = ms_2;
			} else if (key2_state == 0 && !encoder_2_key_encoder_1_key_long_state
					&& (ms_2 - encoder_2_key_time) > 2000) {
				encoder_2_key_encoder_1_key_long_state = 1;

			} else if (key2_state == 1 && encoder_2_key_encoder_1_key_short_state
					&& (ms_2 - encoder_2_key_time) > 50) {
				encoder_2_key_encoder_1_key_short_state = 0;
				encoder_2_key_time = ms_2;
				if (!encoder_2_key_encoder_1_key_long_state) {

					transmit_data[5] = 127;
				}
			}
		}

		//�������� ������ � uart
		if (HAL_GetTick() - tx_time > 100) { //������ 100 ��
			sprintf(message,
					"%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n\r",
					transmit_data[0], transmit_data[1], transmit_data[2],
					transmit_data[3], transmit_data[4], transmit_data[5],
					transmit_data[6], transmit_data[7], transmit_data[8],
					transmit_data[9], transmit_data[10], transmit_data[11],
					transmit_data[12], transmit_data[13], transmit_data[14],
					transmit_data[15], transmit_data[16], transmit_data[17],
					transmit_data[18], transmit_data[19]);
			HAL_UART_Transmit(&huart1, message, sizeof(message), 100);
			tx_time = HAL_GetTick();
		}

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pins : KEY_Pin KEY_2_Pin */
	GPIO_InitStruct.Pin = KEY_Pin | KEY_2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
