/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define exio_RESET_Pin GPIO_PIN_13
#define exio_RESET_GPIO_Port GPIOC
#define NTC3_ADC2_Pin GPIO_PIN_2
#define NTC3_ADC2_GPIO_Port GPIOA
#define NTC2_ADC3_Pin GPIO_PIN_3
#define NTC2_ADC3_GPIO_Port GPIOA
#define NTC1_ADC4_Pin GPIO_PIN_4
#define NTC1_ADC4_GPIO_Port GPIOA
#define IOUT_ADC5_Pin GPIO_PIN_5
#define IOUT_ADC5_GPIO_Port GPIOA
#define VOUT_ADC6_Pin GPIO_PIN_6
#define VOUT_ADC6_GPIO_Port GPIOA
#define VOUT_ADC7_Pin GPIO_PIN_7
#define VOUT_ADC7_GPIO_Port GPIOA
#define RP_EN_Pin GPIO_PIN_10
#define RP_EN_GPIO_Port GPIOB
#define RN_EN_Pin GPIO_PIN_12
#define RN_EN_GPIO_Port GPIOB
#define PC_EN_Pin GPIO_PIN_13
#define PC_EN_GPIO_Port GPIOB
#define AUX1_EN_Pin GPIO_PIN_14
#define AUX1_EN_GPIO_Port GPIOB
#define AUX2_EN_Pin GPIO_PIN_15
#define AUX2_EN_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_6
#define RS485_TX_GPIO_Port GPIOC
#define RS485_RX_Pin GPIO_PIN_7
#define RS485_RX_GPIO_Port GPIOC
#define RS485_DIR_Pin GPIO_PIN_8
#define RS485_DIR_GPIO_Port GPIOC
#define RS485_EX_DIR_Pin GPIO_PIN_8
#define RS485_EX_DIR_GPIO_Port GPIOA
#define RS485_EX_TX_Pin GPIO_PIN_9
#define RS485_EX_TX_GPIO_Port GPIOA
#define RS485_EX_RX_Pin GPIO_PIN_10
#define RS485_EX_RX_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
