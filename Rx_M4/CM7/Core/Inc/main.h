/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M3_Pin GPIO_PIN_0
#define M3_GPIO_Port GPIOA
#define M4_Pin GPIO_PIN_1
#define M4_GPIO_Port GPIOA
#define M2_Pin GPIO_PIN_2
#define M2_GPIO_Port GPIOA
#define M1_Pin GPIO_PIN_3
#define M1_GPIO_Port GPIOA
#define ENC2_A_Pin GPIO_PIN_9
#define ENC2_A_GPIO_Port GPIOE
#define ENC2_B_Pin GPIO_PIN_11
#define ENC2_B_GPIO_Port GPIOE
#define ENC1_A_Pin GPIO_PIN_12
#define ENC1_A_GPIO_Port GPIOD
#define ENC1_B_Pin GPIO_PIN_13
#define ENC1_B_GPIO_Port GPIOD
#define ENC3_A_Pin GPIO_PIN_6
#define ENC3_A_GPIO_Port GPIOC
#define ENC3_B_Pin GPIO_PIN_7
#define ENC3_B_GPIO_Port GPIOC
#define ENC4_A_Pin GPIO_PIN_4
#define ENC4_A_GPIO_Port GPIOB
#define ENC4_B_Pin GPIO_PIN_5
#define ENC4_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
