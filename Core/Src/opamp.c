/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    opamp.c
 * @brief   This file provides code for the configuration
 *          of the OPAMP instances.
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
#include "opamp.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

OPAMP_HandleTypeDef hopamp3;

/* OPAMP3 init function */
void MX_OPAMP3_Init(void) {

	/* USER CODE BEGIN OPAMP3_Init 0 */

	/* USER CODE END OPAMP3_Init 0 */

	/* USER CODE BEGIN OPAMP3_Init 1 */

	/* USER CODE END OPAMP3_Init 1 */
	hopamp3.Instance = OPAMP3;
	hopamp3.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
	hopamp3.Init.Mode = OPAMP_STANDALONE_MODE;
	hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
	hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO1;
	hopamp3.Init.InternalOutput = DISABLE;
	hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	if (HAL_OPAMP_Init(&hopamp3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN OPAMP3_Init 2 */

	/* USER CODE END OPAMP3_Init 2 */

}

void HAL_OPAMP_MspInit(OPAMP_HandleTypeDef *opampHandle) {

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (opampHandle->Instance == OPAMP3) {
		/* USER CODE BEGIN OPAMP3_MspInit 0 */

		/* USER CODE END OPAMP3_MspInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**OPAMP3 GPIO Configuration
		 PB1     ------> OPAMP3_VOUT
		 PB2     ------> OPAMP3_VINM
		 PB13     ------> OPAMP3_VINP
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USER CODE BEGIN OPAMP3_MspInit 1 */

		/* USER CODE END OPAMP3_MspInit 1 */
	}
}

void HAL_OPAMP_MspDeInit(OPAMP_HandleTypeDef *opampHandle) {

	if (opampHandle->Instance == OPAMP3) {
		/* USER CODE BEGIN OPAMP3_MspDeInit 0 */

		/* USER CODE END OPAMP3_MspDeInit 0 */

		/**OPAMP3 GPIO Configuration
		 PB1     ------> OPAMP3_VOUT
		 PB2     ------> OPAMP3_VINM
		 PB13     ------> OPAMP3_VINP
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13);

		/* USER CODE BEGIN OPAMP3_MspDeInit 1 */

		/* USER CODE END OPAMP3_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
