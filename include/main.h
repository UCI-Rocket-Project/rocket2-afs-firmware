/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#define IMU_INT_Pin GPIO_PIN_13
#define IMU_INT_GPIO_Port GPIOC
#define IMU_RST_Pin GPIO_PIN_14
#define IMU_RST_GPIO_Port GPIOC
#define IMU_BOOT_Pin GPIO_PIN_15
#define IMU_BOOT_GPIO_Port GPIOC
#define MEM_HOLD_Pin GPIO_PIN_0
#define MEM_HOLD_GPIO_Port GPIOC
#define ARMED_Pin GPIO_PIN_1
#define ARMED_GPIO_Port GPIOC
#define MEM_WP_Pin GPIO_PIN_2
#define MEM_WP_GPIO_Port GPIOC
#define DROGUE_FIRE_Pin GPIO_PIN_0
#define DROGUE_FIRE_GPIO_Port GPIOA
#define DROGUE_CONT_Pin GPIO_PIN_1
#define DROGUE_CONT_GPIO_Port GPIOA
#define MAIN_FIRE_Pin GPIO_PIN_2
#define MAIN_FIRE_GPIO_Port GPIOA
#define MAIN_CONT_Pin GPIO_PIN_3
#define MAIN_CONT_GPIO_Port GPIOA
#define MEM_SCK_Pin GPIO_PIN_5
#define MEM_SCK_GPIO_Port GPIOA
#define MEM_MISO_Pin GPIO_PIN_6
#define MEM_MISO_GPIO_Port GPIOA
#define MEM_MOSI_Pin GPIO_PIN_7
#define MEM_MOSI_GPIO_Port GPIOA
#define ALT_CS_Pin GPIO_PIN_0
#define ALT_CS_GPIO_Port GPIOB
#define MEM_CS_Pin GPIO_PIN_1
#define MEM_CS_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_10
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_11
#define IMU_SDA_GPIO_Port GPIOB
#define ALT_SCK_Pin GPIO_PIN_13
#define ALT_SCK_GPIO_Port GPIOB
#define ALT_MISO_Pin GPIO_PIN_14
#define ALT_MISO_GPIO_Port GPIOB
#define ALT_MOSI_Pin GPIO_PIN_15
#define ALT_MOSI_GPIO_Port GPIOB
#define LED_FLIGHT_Pin GPIO_PIN_8
#define LED_FLIGHT_GPIO_Port GPIOC
#define LED_STORAGE_Pin GPIO_PIN_9
#define LED_STORAGE_GPIO_Port GPIOC
#define LED_STANDBY_Pin GPIO_PIN_8
#define LED_STANDBY_GPIO_Port GPIOA
#define LED_ARMED_Pin GPIO_PIN_9
#define LED_ARMED_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define GPS_INT_Pin GPIO_PIN_10
#define GPS_INT_GPIO_Port GPIOC
#define GPS_RST_Pin GPIO_PIN_11
#define GPS_RST_GPIO_Port GPIOC
#define GPS_SCL_Pin GPIO_PIN_6
#define GPS_SCL_GPIO_Port GPIOB
#define GPS_SDA_Pin GPIO_PIN_7
#define GPS_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
