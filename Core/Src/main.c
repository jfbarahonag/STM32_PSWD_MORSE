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
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef enum _fsm_states_ {

	STATE_IDLE,
	STATE_DOT,
	STATE_DASH_1,
	STATE_DASH_2,
	STATE_ACCEPTED,

}fsm_states;

typedef enum _fsm_evts_ {

	EVT_NO_EVT,
	EVT_DOT,
	EVT_DASH,
	EVT_CONFIRM_DOT,
	EVT_PRESSED,

}fsm_evts;

typedef struct _fsm_ {

	uint16_t counter_error;
	uint16_t counter_time;
	fsm_states state;
	fsm_evts evt;
	bool new_evt;

}fsm_t;

fsm_t fsm = {
		.counter_error = 0,
		.counter_time = 0,
		.state = STATE_IDLE,
		.evt = EVT_NO_EVT,
		.new_evt = false,
};

typedef struct _button_ {
	uint8_t counter;
	bool maybe_pressed;
	bool pressed;

}button_t;

button_t button = {
		.counter = 0,
		.maybe_pressed = false,
		.pressed = false,
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DOT_TIME		(	   	 200 		)
#define DASH_TIME		(		1000		)
#define TIME_ALLOWED 	(	DOT_TIME + 300	)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	button.maybe_pressed = true;
}

/**
 * @brief  Period elapsed callback in non-blocking mode each 1mSec
 * @param  htim TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (button.maybe_pressed == true) {
		button.counter++;
	}


	if ( button.counter >= 10 ) {
		button.maybe_pressed = false;
		button.counter = 0;
		button.pressed = true;
		fsm.evt = EVT_PRESSED;
		fsm.new_evt = true;
	}

	if( button.pressed == true ) { /* IDK when clear this flag yet!!! */
		if ( !(GPIOC->IDR & (1<<13)) ) { /* Always pressed */
			fsm.counter_time++;
		} else {

			if ( fsm.counter_time >= DOT_TIME && fsm.counter_time <= TIME_ALLOWED ) {
				button.pressed = false;
				fsm.evt = EVT_CONFIRM_DOT;
				fsm.new_evt = true;
			}

			fsm.counter_time = 0;
		}
	}

	if ( fsm.counter_time == DOT_TIME ) {
		fsm.evt = EVT_DOT;
		fsm.new_evt = true;
	}

	if ( fsm.counter_time == DASH_TIME ) {
		if (fsm.state == STATE_IDLE) {

		} else {
			button.pressed = false;
			fsm.counter_time = 0;
			fsm.evt = EVT_DASH;
			fsm.new_evt = true;
		}
	}



}

void print_current_state ( fsm_t *fsm ) {
	switch (fsm->state) {
	case STATE_IDLE:
		HAL_UART_Transmit(&huart2, (uint8_t*)"STATE_IDLE\n", sizeof("STATE_IDLE\n")-1, 100);
		break;
	case STATE_DOT:
		HAL_UART_Transmit(&huart2, (uint8_t*)"STATE_DOT\n", sizeof("STATE_DOT\n")-1, 100);
		break;
	case STATE_DASH_1:
		HAL_UART_Transmit(&huart2, (uint8_t*)"STATE_DASH_1\n", sizeof("STATE_DASH_1\n")-1, 100);
		break;
	case STATE_DASH_2:
		HAL_UART_Transmit(&huart2, (uint8_t*)"STATE_DASH_2\n", sizeof("STATE_DASH_2\n")-1, 100);
		break;
	case STATE_ACCEPTED:
		HAL_UART_Transmit(&huart2, (uint8_t*)"STATE_ACCEPTED\n", sizeof("STATE_ACCEPTED\n")-1, 100);
		break;
	default:
		HAL_UART_Transmit(&huart2, (uint8_t*)"ERROR\n", sizeof("ERROR\n")-1, 100);
		break;
	}
}

/**
 * @brief  Execute password's fsm
 * @param  fsm FSM handle
 * @retval None
 */
void fsm_run ( fsm_t *fsm ) {
	if ( fsm->new_evt == true ) {
		fsm->new_evt = false;
		switch ( fsm->state ) {
		case STATE_IDLE:
			if ( fsm->evt == EVT_DOT ) {
				fsm->state= STATE_IDLE;
			} else if ( fsm->evt == EVT_DASH ) {
				fsm->state= STATE_IDLE;
			} else if ( fsm->evt == EVT_CONFIRM_DOT ) {
				fsm->state= STATE_DOT;
			}
			print_current_state(fsm);
			break;

		case STATE_DOT:
			if ( fsm->evt == EVT_DASH ) {
				fsm->state= STATE_DASH_1;
			} else if ( fsm->evt == EVT_CONFIRM_DOT ) {
				fsm->state= STATE_IDLE;
			}
			print_current_state(fsm);
			break;

		case STATE_DASH_1:
			if ( fsm->evt == EVT_DASH ) {
				fsm->state= STATE_DASH_2;
			} else if ( fsm->evt == EVT_CONFIRM_DOT ) {
				fsm->state= STATE_IDLE;
			}
			print_current_state(fsm);
			break;

		case STATE_DASH_2:
			if ( fsm->evt == EVT_DASH ) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
				fsm->state = STATE_ACCEPTED;
			} else if ( fsm->evt == EVT_CONFIRM_DOT ) {
				fsm->state = STATE_IDLE;
			}
			print_current_state(fsm);
			break;

		case STATE_ACCEPTED:
			if ( fsm->evt == EVT_PRESSED ) {
				HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
				fsm->state = STATE_IDLE;
				print_current_state(fsm);
			}
			break;

		default: /* ERROR */
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
			while(1);
			break;
		}
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
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	print_current_state(&fsm);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		fsm_run(&fsm);
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

	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_8;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
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
	htim2.Init.Prescaler = 15;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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
