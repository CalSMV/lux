/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_1_Pin GPIO_PIN_5
#define PWM_1_GPIO_Port GPIOI
#define PWM_2_Pin GPIO_PIN_7
#define PWM_2_GPIO_Port GPIOI
#define Enable_1_Pin GPIO_PIN_10
#define Enable_1_GPIO_Port GPIOI
#define PWM_3_Pin GPIO_PIN_6
#define PWM_3_GPIO_Port GPIOI
#define Hall_1_Pin GPIO_PIN_4
#define Hall_1_GPIO_Port GPIOK
#define Hall_2_Pin GPIO_PIN_3
#define Hall_2_GPIO_Port GPIOK
#define Hall_3_Pin GPIO_PIN_15
#define Hall_3_GPIO_Port GPIOJ
#define Brake_Pin GPIO_PIN_4
#define Brake_GPIO_Port GPIOD
#define Enable_2_Pin GPIO_PIN_15
#define Enable_2_GPIO_Port GPIOH
#define Enable_3_Pin GPIO_PIN_1
#define Enable_3_GPIO_Port GPIOI
#define Accel_Pin GPIO_PIN_6
#define Accel_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
