/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    gpio.c
 * @brief   This file provides code for the configuration
 *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, IMU_RST_Pin | IMU_BOOT_Pin | MEM_HOLD_Pin | MEM_WP_Pin | LED_FLIGHT_Pin | LED_STORAGE_Pin | GPS_INT_Pin | GPS_RST_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, DROGUE_FIRE_Pin | MAIN_FIRE_Pin | LED_STANDBY_Pin | LED_ARMED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, ALT_CS_Pin | MEM_CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = IMU_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PCPin PCPin PCPin PCPin
                             PCPin PCPin PCPin PCPin */
    GPIO_InitStruct.Pin = IMU_RST_Pin | IMU_BOOT_Pin | MEM_HOLD_Pin | MEM_WP_Pin | LED_FLIGHT_Pin | LED_STORAGE_Pin | GPS_INT_Pin | GPS_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PtPin */
    GPIO_InitStruct.Pin = ARM_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(ARM_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = DROGUE_FIRE_Pin | MAIN_FIRE_Pin | LED_STANDBY_Pin | LED_ARMED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PAPin PAPin */
    GPIO_InitStruct.Pin = DROGUE_CONT_Pin | MAIN_CONT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PBPin PBPin */
    GPIO_InitStruct.Pin = ALT_CS_Pin | MEM_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
