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
#define PE2_Pin GPIO_PIN_2
#define PE2_GPIO_Port GPIOE
#define PE3_Pin GPIO_PIN_3
#define PE3_GPIO_Port GPIOE
#define PE4_Pin GPIO_PIN_4
#define PE4_GPIO_Port GPIOE
#define PE5_Pin GPIO_PIN_5
#define PE5_GPIO_Port GPIOE
#define PE6_Pin GPIO_PIN_6
#define PE6_GPIO_Port GPIOE
#define PC13_Pin GPIO_PIN_13
#define PC13_GPIO_Port GPIOC
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOH
#define PC0_Pin GPIO_PIN_0
#define PC0_GPIO_Port GPIOC
#define PC1_Pin GPIO_PIN_1
#define PC1_GPIO_Port GPIOC
#define PC2_Pin GPIO_PIN_2
#define PC2_GPIO_Port GPIOC
#define PC3_Pin GPIO_PIN_3
#define PC3_GPIO_Port GPIOC
#define PA0_Pin GPIO_PIN_0
#define PA0_GPIO_Port GPIOA
#define PA1_Pin GPIO_PIN_1
#define PA1_GPIO_Port GPIOA
#define PA2_Pin GPIO_PIN_2
#define PA2_GPIO_Port GPIOA
#define PA3_Pin GPIO_PIN_3
#define PA3_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_0
#define LED_GREEN_GPIO_Port GPIOB
#define PB1_Pin GPIO_PIN_1
#define PB1_GPIO_Port GPIOB
#define PB2_Pin GPIO_PIN_2
#define PB2_GPIO_Port GPIOB
#define RST_Pin GPIO_PIN_9
#define RST_GPIO_Port GPIOE
#define SPI4_CS_Pin GPIO_PIN_11
#define SPI4_CS_GPIO_Port GPIOE
#define USB_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_FS_PWR_EN_GPIO_Port GPIOD
#define PD12_Pin GPIO_PIN_12
#define PD12_GPIO_Port GPIOD
#define PD13_Pin GPIO_PIN_13
#define PD13_GPIO_Port GPIOD
#define PD14_Pin GPIO_PIN_14
#define PD14_GPIO_Port GPIOD
#define PD15_Pin GPIO_PIN_15
#define PD15_GPIO_Port GPIOD
#define USB_FS_OVCR_Pin GPIO_PIN_7
#define USB_FS_OVCR_GPIO_Port GPIOG
#define PC6_Pin GPIO_PIN_6
#define PC6_GPIO_Port GPIOC
#define PC7_Pin GPIO_PIN_7
#define PC7_GPIO_Port GPIOC
#define PC8_Pin GPIO_PIN_8
#define PC8_GPIO_Port GPIOC
#define PC9_Pin GPIO_PIN_9
#define PC9_GPIO_Port GPIOC
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define PC10_Pin GPIO_PIN_10
#define PC10_GPIO_Port GPIOC
#define PC11_Pin GPIO_PIN_11
#define PC11_GPIO_Port GPIOC
#define PC12_Pin GPIO_PIN_12
#define PC12_GPIO_Port GPIOC
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_TXD0_Pin GPIO_PIN_13
#define RMII_TXD0_GPIO_Port GPIOG
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PB5_Pin GPIO_PIN_5
#define PB5_GPIO_Port GPIOB
#define PB6_Pin GPIO_PIN_6
#define PB6_GPIO_Port GPIOB
#define PB7_Pin GPIO_PIN_7
#define PB7_GPIO_Port GPIOB
#define PB8_Pin GPIO_PIN_8
#define PB8_GPIO_Port GPIOB
#define PB9_Pin GPIO_PIN_9
#define PB9_GPIO_Port GPIOB
#define PE0_Pin GPIO_PIN_0
#define PE0_GPIO_Port GPIOE
#define PE1_Pin GPIO_PIN_1
#define PE1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
