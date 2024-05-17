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
#define BUZZER_Pin GPIO_PIN_14
#define BUZZER_GPIO_Port GPIOC
#define DROGUE_Pin GPIO_PIN_0
#define DROGUE_GPIO_Port GPIOC
#define DROGUE_CONT_Pin GPIO_PIN_1
#define DROGUE_CONT_GPIO_Port GPIOC
#define MAIN_Pin GPIO_PIN_3
#define MAIN_GPIO_Port GPIOC
#define IMU_INT3_Pin GPIO_PIN_0
#define IMU_INT3_GPIO_Port GPIOA
#define MAIN_CONT_Pin GPIO_PIN_1
#define MAIN_CONT_GPIO_Port GPIOA
#define ARM_CONT_Pin GPIO_PIN_2
#define ARM_CONT_GPIO_Port GPIOA
#define IMU_INT4_Pin GPIO_PIN_4
#define IMU_INT4_GPIO_Port GPIOA
#define IMU_SCK_Pin GPIO_PIN_5
#define IMU_SCK_GPIO_Port GPIOA
#define IMU_MISO_Pin GPIO_PIN_6
#define IMU_MISO_GPIO_Port GPIOA
#define IMU_MOSI_Pin GPIO_PIN_7
#define IMU_MOSI_GPIO_Port GPIOA
#define IMU_CS1_Pin GPIO_PIN_4
#define IMU_CS1_GPIO_Port GPIOC
#define IMU_INT1_Pin GPIO_PIN_5
#define IMU_INT1_GPIO_Port GPIOC
#define IMU_CS2_Pin GPIO_PIN_0
#define IMU_CS2_GPIO_Port GPIOB
#define IMU_INT2_Pin GPIO_PIN_1
#define IMU_INT2_GPIO_Port GPIOB
#define MAG_INT_Pin GPIO_PIN_2
#define MAG_INT_GPIO_Port GPIOB
#define MAG_SCL_Pin GPIO_PIN_10
#define MAG_SCL_GPIO_Port GPIOB
#define MAG_SDA_Pin GPIO_PIN_11
#define MAG_SDA_GPIO_Port GPIOB
#define MEM_CS_Pin GPIO_PIN_12
#define MEM_CS_GPIO_Port GPIOB
#define MEM_SCK_Pin GPIO_PIN_13
#define MEM_SCK_GPIO_Port GPIOB
#define MEM_MISO_Pin GPIO_PIN_14
#define MEM_MISO_GPIO_Port GPIOB
#define MEM_MOSI_Pin GPIO_PIN_15
#define MEM_MOSI_GPIO_Port GPIOB
#define MAG_DRDY_Pin GPIO_PIN_6
#define MAG_DRDY_GPIO_Port GPIOC
#define MEM_WP_Pin GPIO_PIN_7
#define MEM_WP_GPIO_Port GPIOC
#define MEM_HOLD_Pin GPIO_PIN_8
#define MEM_HOLD_GPIO_Port GPIOC
#define LED_ARMED_Pin GPIO_PIN_9
#define LED_ARMED_GPIO_Port GPIOC
#define LED_FLIGHT_Pin GPIO_PIN_8
#define LED_FLIGHT_GPIO_Port GPIOA
#define LED_STANDBY_Pin GPIO_PIN_10
#define LED_STANDBY_GPIO_Port GPIOA
#define LED_STORAGE_Pin GPIO_PIN_15
#define LED_STORAGE_GPIO_Port GPIOA
#define ALT_SCK_Pin GPIO_PIN_10
#define ALT_SCK_GPIO_Port GPIOC
#define ALT_MISO_Pin GPIO_PIN_11
#define ALT_MISO_GPIO_Port GPIOC
#define ALT_MOSI_Pin GPIO_PIN_12
#define ALT_MOSI_GPIO_Port GPIOC
#define ALT_CS_Pin GPIO_PIN_2
#define ALT_CS_GPIO_Port GPIOD
#define GPS_SAFEBOOT_Pin GPIO_PIN_5
#define GPS_SAFEBOOT_GPIO_Port GPIOB
#define GPS_SCL_Pin GPIO_PIN_6
#define GPS_SCL_GPIO_Port GPIOB
#define GPS_SDA_Pin GPIO_PIN_7
#define GPS_SDA_GPIO_Port GPIOB
#define GPS_RESET_Pin GPIO_PIN_8
#define GPS_RESET_GPIO_Port GPIOB
#define GPS_INT_Pin GPIO_PIN_9
#define GPS_INT_GPIO_Port GPIOB
#define USB_VBUS_GPIO_Port GPIOA
#define USB_VBUS_Pin GPIO_PIN_9


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
