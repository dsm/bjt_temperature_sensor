/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "opamp.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdint.h"
#include "stdio.h"
#include "math.h"
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint32_t Vf_raw = 0;
static uint32_t timestamp_ms = 0;
static uint32_t DAC_Value = 600;

static double temp_C = 0;

static double Vf_filtered = 0.0;
static double Vf_filtered_pre = 0.0;

static double filter_coeff = 0.1;
static uint8_t rx_buffer[10] = { 0 };
static uint8_t rx_buffer_counter = 0;

static double coeff0 = -0.057718564039213;
static double coeff1 = 166.0166993386221;

extern UART_HandleTypeDef huart1;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch) {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the LPUART1 and Loop until the end of transmission */
	//HAL_UART_Transmit(&hlpuart1, (uint8_t*) &ch, 1, 0xFFFF);
	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (rx_buffer_counter >= 4) {
		rx_buffer_counter = 0;
		if ('R' == rx_buffer[0] && 'T' == rx_buffer[1] && '\r' == rx_buffer[2]
				&& '\n' == rx_buffer[3]) {
			uint16_t tempC_tx = lround(temp_C * 100.0);
			HAL_UART_Transmit_IT(&huart1, (uint8_t*) &tempC_tx, 2);
		} else {
			uint16_t tempC_tx = 0xFFFF;
			HAL_UART_Transmit_IT(&huart1, (uint8_t*) &tempC_tx, 2);
		}
	}

	if ('R' != rx_buffer[0]) {
		rx_buffer_counter = 0;
		uint16_t tempC_tx = 0xFFFF;
		HAL_UART_Transmit_IT(&huart1, (uint8_t*) &tempC_tx, 2);
	}

	HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_buffer_counter++], 1);

}

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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_DAC1_Init();
	MX_OPAMP3_Init();
	/* USER CODE BEGIN 2 */
	/* Run the ADC calibration in single-ended mode */

	/* Enable OPAMP */
	if (HAL_OK != HAL_OPAMP_Start(&hopamp3)) {
		Error_Handler();
	}

	if (HAL_OK != HAL_ADC_Start_DMA(&hadc1, &Vf_raw, 1)) {
		/* Start DMA Error */
		Error_Handler();
	}
	HAL_UART_Receive_IT(&huart1, &rx_buffer[rx_buffer_counter++], 1);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, DAC_Value);
	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
	HAL_Delay(10);
	Vf_filtered_pre = (Vf_raw * 0.800);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		if ((HAL_GetTick() - timestamp_ms) >= 100) {
			timestamp_ms = HAL_GetTick();

			HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R,
					DAC_Value);

			Vf_filtered = filter_coeff * (Vf_raw * 0.800)
					+ (1.0 - filter_coeff) * Vf_filtered_pre;
			Vf_filtered_pre = Vf_filtered;

			temp_C = coeff0 * Vf_filtered + coeff1;

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
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
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_Delay(500);
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
