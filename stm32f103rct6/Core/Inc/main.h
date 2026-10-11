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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_14
#define LED_G_GPIO_Port GPIOC
#define LED_R_Pin GPIO_PIN_15
#define LED_R_GPIO_Port GPIOC
#define RS485_RE_Pin GPIO_PIN_0
#define RS485_RE_GPIO_Port GPIOC
#define LED_WS2812_Pin GPIO_PIN_1
#define LED_WS2812_GPIO_Port GPIOC
#define IIC_SDA_Pin GPIO_PIN_2
#define IIC_SDA_GPIO_Port GPIOC
#define IIC_SCL_Pin GPIO_PIN_3
#define IIC_SCL_GPIO_Port GPIOC
#define TIM5_CH1_Pin GPIO_PIN_0
#define TIM5_CH1_GPIO_Port GPIOA
#define TIM5_CH2_Pin GPIO_PIN_1
#define TIM5_CH2_GPIO_Port GPIOA
#define U2_TX_Pin GPIO_PIN_2
#define U2_TX_GPIO_Port GPIOA
#define U2_RX_Pin GPIO_PIN_3
#define U2_RX_GPIO_Port GPIOA
#define LASER_REV_Pin GPIO_PIN_4
#define LASER_REV_GPIO_Port GPIOA
#define LASER_REV_EXTI_IRQn EXTI4_IRQn
#define KEY_2_Pin GPIO_PIN_6
#define KEY_2_GPIO_Port GPIOA
#define AD_H_Y_Pin GPIO_PIN_4
#define AD_H_Y_GPIO_Port GPIOC
#define AD_BAT_Pin GPIO_PIN_5
#define AD_BAT_GPIO_Port GPIOC
#define AD_H_X_Pin GPIO_PIN_0
#define AD_H_X_GPIO_Port GPIOB
#define AD_RES_Pin GPIO_PIN_1
#define AD_RES_GPIO_Port GPIOB
#define KEY_1_Pin GPIO_PIN_2
#define KEY_1_GPIO_Port GPIOB
#define U3_TX_Pin GPIO_PIN_10
#define U3_TX_GPIO_Port GPIOB
#define U3_RX_Pin GPIO_PIN_11
#define U3_RX_GPIO_Port GPIOB
#define KEY_H_Pin GPIO_PIN_12
#define KEY_H_GPIO_Port GPIOB
#define SPI2_SCK_Pin GPIO_PIN_13
#define SPI2_SCK_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define SPI2_NSS_Pin GPIO_PIN_8
#define SPI2_NSS_GPIO_Port GPIOC
#define SENS2_Pin GPIO_PIN_8
#define SENS2_GPIO_Port GPIOA
#define SENS4_Pin GPIO_PIN_9
#define SENS4_GPIO_Port GPIOA
#define SENS1_Pin GPIO_PIN_10
#define SENS1_GPIO_Port GPIOA
#define SENS3_Pin GPIO_PIN_11
#define SENS3_GPIO_Port GPIOA
#define U4_TX_Pin GPIO_PIN_10
#define U4_TX_GPIO_Port GPIOC
#define U5_TX_Pin GPIO_PIN_12
#define U5_TX_GPIO_Port GPIOC
#define U5_RX_Pin GPIO_PIN_2
#define U5_RX_GPIO_Port GPIOD
#define LASER_TRIG_Pin GPIO_PIN_5
#define LASER_TRIG_GPIO_Port GPIOB
#define U1_TX_BUZ_Pin GPIO_PIN_6
#define U1_TX_BUZ_GPIO_Port GPIOB
#define U1_RX_Pin GPIO_PIN_7
#define U1_RX_GPIO_Port GPIOB
#define CAN_RX_Pin GPIO_PIN_8
#define CAN_RX_GPIO_Port GPIOB
#define CAN_TX_Pin GPIO_PIN_9
#define CAN_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
