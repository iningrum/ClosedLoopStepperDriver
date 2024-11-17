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
#include "stm32g4xx_hal.h"

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
#define LED_WS_Pin GPIO_PIN_13
#define LED_WS_GPIO_Port GPIOC
#define LED_OK_Pin GPIO_PIN_14
#define LED_OK_GPIO_Port GPIOC
#define LED_FAULT_Pin GPIO_PIN_15
#define LED_FAULT_GPIO_Port GPIOC
#define DRV_DIR_Pin GPIO_PIN_0
#define DRV_DIR_GPIO_Port GPIOA
#define DRV_RESET_Pin GPIO_PIN_2
#define DRV_RESET_GPIO_Port GPIOA
#define DRV_SLEEPn_Pin GPIO_PIN_3
#define DRV_SLEEPn_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_4
#define DRV_CS_GPIO_Port GPIOA
#define DRV_SCK_Pin GPIO_PIN_5
#define DRV_SCK_GPIO_Port GPIOA
#define DRV_MISO_Pin GPIO_PIN_6
#define DRV_MISO_GPIO_Port GPIOA
#define DRV_MOSI_Pin GPIO_PIN_7
#define DRV_MOSI_GPIO_Port GPIOA
#define CURR_MEAS_Pin GPIO_PIN_0
#define CURR_MEAS_GPIO_Port GPIOB
#define TEMP_MEAS_Pin GPIO_PIN_1
#define TEMP_MEAS_GPIO_Port GPIOB
#define DRV_STALLn_Pin GPIO_PIN_2
#define DRV_STALLn_GPIO_Port GPIOB
#define SW3_Pin GPIO_PIN_10
#define SW3_GPIO_Port GPIOB
#define ENC_CS_Pin GPIO_PIN_12
#define ENC_CS_GPIO_Port GPIOB
#define ENC_SCK_Pin GPIO_PIN_13
#define ENC_SCK_GPIO_Port GPIOB
#define ROM_WP_Pin GPIO_PIN_14
#define ROM_WP_GPIO_Port GPIOB
#define ENC_MOSI_Pin GPIO_PIN_15
#define ENC_MOSI_GPIO_Port GPIOB
#define ROM_SDA_Pin GPIO_PIN_8
#define ROM_SDA_GPIO_Port GPIOA
#define ROM_SCL_Pin GPIO_PIN_9
#define ROM_SCL_GPIO_Port GPIOA
#define CAN_SHDN_Pin GPIO_PIN_10
#define CAN_SHDN_GPIO_Port GPIOA
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOB
#define DRV_FAULTn_Pin GPIO_PIN_6
#define DRV_FAULTn_GPIO_Port GPIOB
#define END_SWITCH_1_Pin GPIO_PIN_7
#define END_SWITCH_1_GPIO_Port GPIOB
#define END_SWITCH_2_Pin GPIO_PIN_9
#define END_SWITCH_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
