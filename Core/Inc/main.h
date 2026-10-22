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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BLE_STATE_Pin GPIO_PIN_1
#define BLE_STATE_GPIO_Port GPIOA
#define BLE_TX_Pin GPIO_PIN_2
#define BLE_TX_GPIO_Port GPIOA
#define BLE_RX_Pin GPIO_PIN_3
#define BLE_RX_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_4
#define KEY2_GPIO_Port GPIOA
#define KEY3_Pin GPIO_PIN_5
#define KEY3_GPIO_Port GPIOA
#define E2A_TIM_Pin GPIO_PIN_6
#define E2A_TIM_GPIO_Port GPIOA
#define E2B_TIM_Pin GPIO_PIN_7
#define E2B_TIM_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOB
#define BUZZ_Pin GPIO_PIN_1
#define BUZZ_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_10
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_11
#define OLED_SDA_GPIO_Port GPIOB
#define M_AIN2_Pin GPIO_PIN_12
#define M_AIN2_GPIO_Port GPIOB
#define M_AIN1_Pin GPIO_PIN_13
#define M_AIN1_GPIO_Port GPIOB
#define M_BIN1_Pin GPIO_PIN_14
#define M_BIN1_GPIO_Port GPIOB
#define M_BIN2_Pin GPIO_PIN_15
#define M_BIN2_GPIO_Port GPIOB
#define M_PWMB_Pin GPIO_PIN_8
#define M_PWMB_GPIO_Port GPIOA
#define M_PWMA_Pin GPIO_PIN_9
#define M_PWMA_GPIO_Port GPIOA
#define M_STBY_Pin GPIO_PIN_10
#define M_STBY_GPIO_Port GPIOA
#define BLE_EN_Pin GPIO_PIN_3
#define BLE_EN_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_5
#define LED1_GPIO_Port GPIOB
#define E1A_TIM_Pin GPIO_PIN_6
#define E1A_TIM_GPIO_Port GPIOB
#define E1B_TIM_Pin GPIO_PIN_7
#define E1B_TIM_GPIO_Port GPIOB
#define GS_SCL_Pin GPIO_PIN_8
#define GS_SCL_GPIO_Port GPIOB
#define GS_SDA_Pin GPIO_PIN_9
#define GS_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
