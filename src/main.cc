#include "main.h"

#include "cmath"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gps.h"

void SystemClock_Config(void);
void initializeM8QI2C();


int main(void) {
    HAL_Init();
    initializeM8QI2C();
    HAL_I2C_Init(&hi2c1);

    SystemClock_Config();

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();

    HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_FLIGHT_GPIO_Port, LED_FLIGHT_Pin, GPIO_PIN_RESET);

    // HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, 0x42<<1, 10, 1000);
    
    // HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, 0x42<<1, (uint8_t*)"\x00\x00", 2, 1000);
    UBX_NAV_STATUS nav_status_buffer;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, 0x42, 0x03, 16, (uint8_t*)&nav_status_buffer, 16, 1000);

    // HAL_StatusTypeDef status = HAL_I2C_Master_Receive(&hi2c1, 0x42<<1, (uint8_t*)"\x00\x00", 2, 1000);

    HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_RESET);
    if(status != HAL_OK) {
        HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_FLIGHT_GPIO_Port, LED_FLIGHT_Pin, GPIO_PIN_SET);
    }
}

void initializeM8QI2C() {
    hi2c1.Instance = I2C2;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    hi2c1.Memaddress = 0x01;
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_PLLI2S_ENABLE();
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
    }
}
