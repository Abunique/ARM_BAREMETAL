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
#include "stm32h5xx_hal.h"

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
#define ON_OFF_button_Pin GPIO_PIN_13
#define ON_OFF_button_GPIO_Port GPIOC
#define ON_OFF_button_EXTI_IRQn EXTI13_IRQn
#define SEQUENCE_SRT_STP_Pin GPIO_PIN_14
#define SEQUENCE_SRT_STP_GPIO_Port GPIOC
#define SEQUENCE_SRT_STP_EXTI_IRQn EXTI14_IRQn
#define IDLE_ALL_OFF_Pin GPIO_PIN_15
#define IDLE_ALL_OFF_GPIO_Port GPIOC
#define IDLE_ALL_OFF_EXTI_IRQn EXTI15_IRQn
#define ERR_BUTTON_Pin GPIO_PIN_0
#define ERR_BUTTON_GPIO_Port GPIOF
#define ERR_BUTTON_EXTI_IRQn EXTI0_IRQn
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOF
#define LED1_Pin GPIO_PIN_0
#define LED1_GPIO_Port GPIOB
#define LED4_NA_Pin GPIO_PIN_3
#define LED4_NA_GPIO_Port GPIOG
#define LED3_Pin GPIO_PIN_4
#define LED3_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
