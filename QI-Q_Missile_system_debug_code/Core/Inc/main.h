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
#define WEITIAO_Pin GPIO_PIN_13
#define WEITIAO_GPIO_Port GPIOC
#define CUTIAO_Pin GPIO_PIN_14
#define CUTIAO_GPIO_Port GPIOC
#define FORCE_WORKING_Pin GPIO_PIN_6
#define FORCE_WORKING_GPIO_Port GPIOC
#define YAW_WORKING_Pin GPIO_PIN_7
#define YAW_WORKING_GPIO_Port GPIOC
#define CAN_WORKING_Pin GPIO_PIN_8
#define CAN_WORKING_GPIO_Port GPIOC
#define POWER_SUPPLY_Pin GPIO_PIN_9
#define POWER_SUPPLY_GPIO_Port GPIOC
#define CUSTOM_Pin GPIO_PIN_8
#define CUSTOM_GPIO_Port GPIOA
#define YAW_K_Pin GPIO_PIN_8
#define YAW_K_GPIO_Port GPIOB
#define FORCE_K_Pin GPIO_PIN_9
#define FORCE_K_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
