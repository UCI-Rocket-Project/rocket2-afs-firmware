#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "gps.h"

void SystemClock_Config(void);

typedef GpsUbxM8I2c GPS;


int main(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    MX_I2C1_Init();

    // write some garbage to i2c1
    uint8_t garbage[2] = {0x0A, 0x0B};

    while(1) {
        HAL_I2C_Master_Transmit(&hi2c1, 0x42 << 1, garbage, 2, 100);
        HAL_Delay(1000);
        
        // toggle flight led
        HAL_GPIO_TogglePin(LED_FLIGHT_GPIO_Port, LED_FLIGHT_Pin);
    }

    return 0;

    /* USER CODE BEGIN 2 */
    GPS gps(GPS_RST_GPIO_Port, GPS_RST_Pin, &hi2c1, PVT_MESSAGE);
    gps.Init();
    HAL_Delay(1000);

    int lastITOW = 0;
    while(1) {
        volatile GPS::PollResult res = gps.PollUpdate();    
        auto state = gps.GetState();
        if(state == GPS::State::RESPONSE_READY) {
            UBX_NAV_PVT_PAYLOAD sol = *(UBX_NAV_PVT_PAYLOAD*)gps.GetSolution();            
            volatile int diff = sol.iTOW - lastITOW;
            lastITOW = sol.iTOW;
            if((GPSFixType)sol.fixType == GPSFixType::FIX_3D) {
                HAL_Delay(1000);
            }
            gps.Reset();
        } else {
            HAL_Delay(100);
        }
    }
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