/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define RIGHT_FRONT_LED_Pin GPIO_PIN_10
#define RIGHT_FRONT_LED_GPIO_Port GPIOG
#define LEFT_FRONT_LED_Pin GPIO_PIN_5
#define LEFT_FRONT_LED_GPIO_Port GPIOD
#define HIGH_BEAM_Pin GPIO_PIN_0
#define HIGH_BEAM_GPIO_Port GPIOI
#define RIGHT_REAL_LED_Pin GPIO_PIN_3
#define RIGHT_REAL_LED_GPIO_Port GPIOI
#define LEFT_REAL_LED_Pin GPIO_PIN_11
#define LEFT_REAL_LED_GPIO_Port GPIOG
#define TURN_EN_Pin GPIO_PIN_6
#define TURN_EN_GPIO_Port GPIOD
#define LOW_BEAM_Pin GPIO_PIN_1
#define LOW_BEAM_GPIO_Port GPIOI
#define DRIVE_EN_Pin GPIO_PIN_9
#define DRIVE_EN_GPIO_Port GPIOG
#define BAT_V_ADC_Pin GPIO_PIN_6
#define BAT_V_ADC_GPIO_Port GPIOA
#define BAT_I_ADC_Pin GPIO_PIN_11
#define BAT_I_ADC_GPIO_Port GPIOF

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
