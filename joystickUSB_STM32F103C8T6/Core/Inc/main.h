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
#include "stm32f1xx_hal.h"

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
#define IN_DB9_PIN9_Pin GPIO_PIN_6
#define IN_DB9_PIN9_GPIO_Port GPIOA
#define IN_DB9_PIN6_Pin GPIO_PIN_7
#define IN_DB9_PIN6_GPIO_Port GPIOA
#define IN_DB9_PIN4_Pin GPIO_PIN_0
#define IN_DB9_PIN4_GPIO_Port GPIOB
#define IN_DB9_PIN3_Pin GPIO_PIN_1
#define IN_DB9_PIN3_GPIO_Port GPIOB
#define IN_DB9_PIN2_Pin GPIO_PIN_10
#define IN_DB9_PIN2_GPIO_Port GPIOB
#define IN_DB9_PIN1_Pin GPIO_PIN_11
#define IN_DB9_PIN1_GPIO_Port GPIOB
#define OUT_DB9_PIN7_Pin GPIO_PIN_3
#define OUT_DB9_PIN7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
