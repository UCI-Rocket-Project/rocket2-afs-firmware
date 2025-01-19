/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "altimeter_ms5607_spi.h"
#include "gps_ubxm8_i2c.h"
#include "imu_bmi088_spi.h"
#include "magnetometer_bmi150_i2c.h"
#include "memory_w25q1128jv_spi.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
/* USER CODE BEGIN PV */
uint8_t usbCommand;

/* Memory initalised */
MemoryW25q1128jvSpi memory(&hspi2, MEM_CS_GPIO_Port, MEM_CS_Pin, MEM_WP_GPIO_Port, MEM_WP_Pin, MEM_HOLD_GPIO_Port, MEM_HOLD_Pin);

/* Altimeter initialised */
AltimeterMs5607Spi altimeter(&hspi3, ALT_CS_GPIO_Port, ALT_CS_Pin, ALT_MISO_GPIO_Port, ALT_MISO_Pin, 1014.9, 100);

/* GPS initialised */
GpsUbxM8I2c gps(GPS_RESET_GPIO_Port, GPS_RESET_Pin, &hi2c1, PVT_MESSAGE);

/* IMU initialised */
ImuBmi088Spi imu(&hspi1, IMU_CS1_GPIO_Port, IMU_CS1_Pin, IMU_CS2_GPIO_Port, IMU_CS2_Pin);

/* Magnetometer initialised */
MagBmi150i2c magnetometer(&hi2c2, MAG_INT_GPIO_Port, MAG_INT_Pin, MAG_DRDY_GPIO_Port, MAG_DRDY_Pin);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
    /* USER CODE BEGIN 1 */
    // Memory Data packages
    MemoryW25q1128jvSpi::AfsTelemetryData afsData;
    MemoryW25q1128jvSpi::AfsState afsState;

    // Altimeter data package
    AltimeterMs5607Spi::Data altData;
    AltimeterMs5607Spi::State altState;

    // GPS data package
    UBX_NAV_PVT_PAYLOAD gpsData;

    // IMU data package
    ImuBmi088Spi::Data imuData;

    // Magnetometer data package
    MagBmi150i2c::Data magData;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_SPI1_Init();
    MX_SPI2_Init();
    MX_SPI3_Init();
    MX_USB_DEVICE_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();

    /* HAL timer starts */
    HAL_TIM_Base_Start(&htim4);
    HAL_TIM_Base_Start(&htim5);

    /* Sensor module resets */
    altimeter.Reset();
    imu.Reset();
    magnetometer.Reset();
    gps.Reset();
    HAL_Delay(100);  // Ensures reset done for all modules

    /******************** USB call ********************/
    uint8_t usbBuffer[64];
    char text[256];
    int retval;

    while (1) {
        // breaks loop if usb not plugged in
        if (HAL_GPIO_ReadPin(USB_VBUS_GPIO_Port, USB_VBUS_Pin) != GPIO_PIN_SET) break;

        switch (usbCommand) {
            // Erases memory
            // Note: for some reason, VBUS goes low and the loop ends
            case (0x4F):
                // Set LEDs
                HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_FLIGHT_GPIO_Port, LED_FLIGHT_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_STORAGE_GPIO_Port, LED_STORAGE_Pin, GPIO_PIN_SET);

                memory.ChipErase();

                sprintf(text, "Memory fully erased\n");
                CDC_Transmit_FS((uint8_t *)text, strlen(text));

                // reset usbCommand
                usbCommand = 0;
                break;

            // USB Memory dump of all nodes
            case (0x68):
                // Reduce frequency so hterm doesn't crash lol
                HAL_GPIO_TogglePin(LED_STORAGE_GPIO_Port, LED_STORAGE_Pin);
                // gets Memory data
                retval = memory.ChipReadDump(usbBuffer);
                // error handler
                if (retval == -1) {
                    sprintf(text, "Memory SPI error\n");
                    while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                    }
                    usbCommand = 0;
                    break;
                } else if (retval == -2) {
                    sprintf(text, "Memory is full and is fully dumped\n");
                    while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                    }
                    // reset usb command being sent
                    usbCommand = 0;
                    break;
                } else if (usbBuffer[0] != 0x00) {
                    sprintf(text, "Memory fully dumped\n");
                    while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                    }
                    // reset usb command being sent
                    usbCommand = 0;
                    break;
                }

                afsData = *(MemoryW25q1128jvSpi::AfsTelemetryData *)&usbBuffer;
                afsState = *(MemoryW25q1128jvSpi::AfsState *)&afsData.state;

                sprintf(text, "Address: %06X, ", retval);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "Timestamp: %010d, ", afsData.timestamp);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "State: %01X, ArmCont: %01X, DrogueCont: %01X, MainCont: %01X, ", afsState.state, afsState.armPinState, afsState.drogueContinuity, afsState.mainContinuity);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "AngV_X: %05d, AngV_Y: %05d, AngV_Z: %05d, ", afsData.angularVelocityX, afsData.angularVelocityY, afsData.angularVelocityZ);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "AccX: %05d, AccY: %05d, AccZ: %05d, ", afsData.accelerationX, afsData.accelerationY, afsData.accelerationZ);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "MagX: %05d, MagY: %05d, MagZ: %05d, ", afsData.magneticFieldX, afsData.magneticFieldY, afsData.magneticFieldZ);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "Temp: %05d, Alt: %09d, ", afsData.temperature, (int)afsData.altitude);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "GPSposX: %010d, GPSposY: %010d, GPSposZ: %010d, GPSposAcc: %010d, ", afsData.ecefPositionX, afsData.ecefPositionY, afsData.ecefPositionZ, afsData.ecefPositionAccuracy);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "GPSvelX: %010d, GPSvelY: %011d, GPSvelZ: %010d, GPSvelAcc: %010d, ", afsData.ecefVelocityX, afsData.ecefVelocityY, afsData.ecefVelocityZ, afsData.ecefVelocityAccuracy);
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }

                sprintf(text, "\r\n");
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }
                break;

            default:
                HAL_Delay(1000);
                HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_FLIGHT_GPIO_Port, LED_FLIGHT_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_STORAGE_GPIO_Port, LED_STORAGE_Pin, GPIO_PIN_SET);

                // test usb send from AFS to confirm connection
                sprintf(text, "1\r");
                while (CDC_Transmit_FS((uint8_t *)text, strlen(text)) == USBD_BUSY) {
                }
                break;
        }
    }

    /******************** MAIN AFS FIRMWARE LOOP VARIABLES ********************/
    /* Sensor module initialises */
    // memory.ChipErase();
    memory.Init();
    altimeter.Init();
    imu.Init();
    gps.Init();
    magnetometer.Init();
    HAL_Delay(1000);

    uint32_t gpsPreviousTow = 0;

    // Buffer array to write into flash memory
    uint8_t memoryBuffer[64];

    // Current state of AFS, value = to AfsState struct documentation
    uint8_t state = 0x0;

    // Memory LED blinking counter
    int memoryLEDCounter = 0;

    // Starting altitude
    volatile double startAltitude = 0.0;
    // Averaging to find starting alt
    double j = 0.0;
    while (j < 10) {
        altState = altimeter.Read(AltimeterMs5607Spi::Rate::OSR4096);
        if (altState == AltimeterMs5607Spi::State::COMPLETE) {
            altData = altimeter.GetData();
            startAltitude += altData.altitude;
            j++;
        }
    }
    // Starting altitude averaged out
    startAltitude = startAltitude / j;

    // Previous altitude to detect altitude changes over time
    double prevAltitude = startAltitude;

    // Counter for continuous ticks for true finite state machine state changes
    int tick = 0;

    // Previous time for timing delay of writing into
    uint32_t prevTime = TIM5->CNT << 16 | TIM4->CNT;

    while (1) {
        /******************** DATA COLLECTION ********************/
        afsData.type = 0x00;
        // uint32_t timeStamp = HAL_GetTick();
        uint32_t timeStamp = TIM5->CNT << 16 | TIM4->CNT;
        afsData.timestamp = timeStamp;

        afsState.armPinState = HAL_GPIO_ReadPin(ARM_CONT_GPIO_Port, ARM_CONT_Pin);
        afsState.drogueContinuity = HAL_GPIO_ReadPin(DROGUE_CONT_GPIO_Port, DROGUE_CONT_Pin);
        afsState.mainContinuity = HAL_GPIO_ReadPin(MAIN_CONT_GPIO_Port, MAIN_CONT_Pin);
        afsState.state = state;
        afsData.state = *(uint8_t *)&afsState;

        /* Altimeter data */
        altState = altimeter.Read(AltimeterMs5607Spi::Rate::OSR4096);
        if (altState == AltimeterMs5607Spi::State::COMPLETE) {
            altData = altimeter.GetData();
            afsData.temperature = altData.temperature;
            afsData.altitude = altData.altitude;
        }

        /* GPS Data */
        gps.PollUpdate();
        GpsUbxM8I2c::State gpsState = gps.GetState();
        if (gpsState == GpsUbxM8I2c::State::RESPONSE_READY) {
            // getting data
            gpsData = *(UBX_NAV_PVT_PAYLOAD *)gps.GetSolution();
            afsData.ecefPositionX = gpsData.ecefX;
            afsData.ecefPositionY = gpsData.ecefY;
            afsData.ecefPositionZ = gpsData.ecefZ;
            afsData.ecefVelocityX = gpsData.ecefVX;
            afsData.ecefVelocityY = gpsData.ecefVY;
            afsData.ecefVelocityZ = gpsData.ecefVZ;
            afsData.ecefPositionAccuracy = gpsData.pAcc;
            afsData.ecefVelocityAccuracy = gpsData.sAcc;

            if ((GPSFixType)gpsData.gpsFix == GPSFixType::FIX_3D && gpsData.iTOW > gpsPreviousTow) {
                HAL_GPIO_TogglePin(LED_FLIGHT_GPIO_Port, LED_FLIGHT_Pin);
                gpsPreviousTow = gpsData.iTOW;
            }
            gps.Reset();
        }

        /* IMU data */
        imuData = imu.Read();
        afsData.angularVelocityX = -imuData.angularVelocityX;
        afsData.angularVelocityY = imuData.angularVelocityY;
        afsData.angularVelocityZ = -imuData.angularVelocityZ;
        afsData.accelerationX = -imuData.accelerationX;
        afsData.accelerationY = imuData.accelerationY;
        afsData.accelerationZ = -imuData.accelerationZ;

        /* Magnetometer data */
        magData = magnetometer.Read();
        afsData.magneticFieldX = magData.magneticFieldY;
        afsData.magneticFieldY = -magData.magneticFieldX;
        afsData.magneticFieldZ = magData.magneticFieldZ;

        /******************** State maching and parachute deployment ********************/
        switch (state) {
            case (0x0):
                /* Standby mode 0x0 */
                // Standby mode, keep collecting data but never log into memory
                HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_FLIGHT_GPIO_Port, LED_FLIGHT_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_STORAGE_GPIO_Port, LED_STORAGE_Pin, GPIO_PIN_RESET);

                if (HAL_GPIO_ReadPin(ARM_CONT_GPIO_Port, ARM_CONT_Pin) == 1) {
                    state = 0x1;
                }
                break;
            case (0x1):
                /* ARMED mode 0x01 */
                // Armed mode: will collect data and look for flught condition
                HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_SET);

                if ((altData.altitude != -1) && altData.altitude != prevAltitude) {
                    if (altData.altitude > startAltitude + 3) {
                        tick++;
                        if (tick > 20) {
                            tick = 0;
                            state = 0x2;
                        }
                    } else {
                        tick = 0;
                    }
                }

                break;

            case (0x2):
                /* BOOST mode 0x02 */
                // when the rocket is in flight off launch
                HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_SET);

                // condition to find when to deploy drogue parachute
                if (afsData.altitude != -1 && altData.altitude != prevAltitude) {
                    if (altData.altitude < prevAltitude) {
                        tick++;
                        // 100 is found based on HZ of loop being around 50 HZ ish
                        if (tick > 5) {
                            tick = 0;
                            state = 0x4;
                        }
                    } else if (altData.altitude > prevAltitude) {
                        // Reset count if altitude is increasing again
                        tick = 0;
                    }
                }

                break;

            case (0x4):
                /* Apogee 0x4 */
                // deploys the drogue parachute about 2 sec after apogee
                HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_RESET);

                HAL_GPIO_WritePin(DROGUE_GPIO_Port, DROGUE_Pin, GPIO_PIN_SET);

                // condition to find when to deploy main parachute
                if ((afsData.altitude != -1) && (altData.altitude != prevAltitude)) {
                    // change state after 5 ticks of AFS under 200 ft or 61 meters
                    if ((altData.altitude < startAltitude + 61)) {
                        tick++;
                        if (tick > 5) {
                            tick = 0;
                            state = 0x8;
                        }
                    } else {
                        // Reset count if altitude is increasing again
                        tick = 0;
                    }
                }

                break;

            case (0x8):
                /* Main 0x8 */
                // Deploys the main parachute about
                HAL_GPIO_WritePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_SET);

                HAL_GPIO_WritePin(MAIN_GPIO_Port, MAIN_Pin, GPIO_PIN_SET);

                // condition to find when the rocket has landed
                if ((afsData.altitude != -1) && (altData.altitude != prevAltitude)) {
                    // altitude is change is less than meter
                    if (abs(altData.altitude - prevAltitude) < 1) {
                        tick++;
                        if (tick > 5) {
                            tick = 0;
                            state = 0xB;
                        }
                    } else {
                        // Reset count if altitude is increaseing again
                        tick = 0;
                    }
                }

            case (0xB):
                /* Land 0xB */
                // Landing conditinon, do nothing but still log data
                HAL_GPIO_TogglePin(LED_STANDBY_GPIO_Port, LED_STANDBY_Pin);
                HAL_GPIO_WritePin(LED_ARMED_GPIO_Port, LED_ARMED_Pin, GPIO_PIN_RESET);

                HAL_Delay(100);

            default:
                break;
        }

        afsData.angularVelocityX = tick;  ///////////////////////////////////////////////////////////

        /********************  Data written into memory ********************/
        memcpy(memoryBuffer, &afsData, sizeof(memoryBuffer));
        // when AFS is armed and on launch rails, store data every .5 seconds
        if (state == 0x1 || state == 0xB) {
            if (HAL_GetTick() > prevTime + 100) {
                if (memory.ChipWrite(memoryBuffer) == MemoryW25q1128jvSpi::State::COMPLETE) {
                    // LED blinking
                    if (memoryLEDCounter % 10 == 0) {
                        // for blinking LED
                        HAL_GPIO_TogglePin(LED_STORAGE_GPIO_Port, LED_STORAGE_Pin);
                    }
                    memoryLEDCounter++;
                    // reestablish the prevTime
                    prevTime = HAL_GetTick();
                    // reset the data from modules with lead time
                    afsData.temperature = 0xFFFF;
                    afsData.altitude = 0xFFFFFFFF;
                    afsData.ecefPositionX = 0xFFFFFFFF;
                    afsData.ecefPositionY = 0xFFFFFFFF;
                    afsData.ecefPositionZ = 0xFFFFFFFF;
                    afsData.ecefVelocityX = 0xFFFFFFFF;
                    afsData.ecefVelocityY = 0xFFFFFFFF;
                    afsData.ecefVelocityZ = 0xFFFFFFFF;
                    afsData.ecefPositionAccuracy = 0xFFFFFFFF;
                    afsData.ecefVelocityAccuracy = 0xFFFFFFFF;
                }
            }
        }
        // when AFS is in flight, store data as fast as possible
        else if (state > 0x1 && state < 0xB) {
            if (memory.ChipWrite(memoryBuffer) == MemoryW25q1128jvSpi::State::COMPLETE) {
                // LED blinking
                if (memoryLEDCounter % 10 == 0) {
                    // for blinking LED
                    HAL_GPIO_TogglePin(LED_STORAGE_GPIO_Port, LED_STORAGE_Pin);
                }
                memoryLEDCounter++;
                // reestabilish the prevTime
                prevTime = HAL_GetTick();
                // reset the data from modules with lead time
                afsData.temperature = 0xFFFF;
                afsData.altitude = 0xFFFFFFFF;
                afsData.ecefPositionX = 0xFFFFFFFF;
                afsData.ecefPositionY = 0xFFFFFFFF;
                afsData.ecefPositionZ = 0xFFFFFFFF;
                afsData.ecefVelocityX = 0xFFFFFFFF;
                afsData.ecefVelocityY = 0xFFFFFFFF;
                afsData.ecefVelocityZ = 0xFFFFFFFF;
                afsData.ecefPositionAccuracy = 0xFFFFFFFF;
                afsData.ecefVelocityAccuracy = 0xFFFFFFFF;
            }
        }

        // update the previous altitude data
        if (altData.altitude != prevAltitude) {
            prevAltitude = altData.altitude;
        }
    }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    /** Configure the Systick interrupt time
     */
    __HAL_RCC_PLLI2S_ENABLE();
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {
    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {
    /* USER CODE BEGIN I2C2_Init 0 */

    /* USER CODE END I2C2_Init 0 */

    /* USER CODE BEGIN I2C2_Init 1 */

    /* USER CODE END I2C2_Init 1 */
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C2_Init 2 */

    /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {
    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {
    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance = SPI2;
    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.Direction = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi2.Init.NSS = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief SPI3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI3_Init(void) {
    /* USER CODE BEGIN SPI3_Init 0 */

    /* USER CODE END SPI3_Init 0 */

    /* USER CODE BEGIN SPI3_Init 1 */

    /* USER CODE END SPI3_Init 1 */
    /* SPI3 parameter configuration*/
    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi3) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI3_Init 2 */

    /* USER CODE END SPI3_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, BUZZER_Pin | DROGUE_Pin | MAIN_Pin | IMU_CS1_Pin | IMU_INT1_Pin | MAG_DRDY_Pin | MEM_WP_Pin | MEM_HOLD_Pin | LED_ARMED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, IMU_INT3_Pin | IMU_INT4_Pin | LED_FLIGHT_Pin | LED_STANDBY_Pin | LED_STORAGE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, IMU_CS2_Pin | IMU_INT2_Pin | MAG_INT_Pin | MEM_CS_Pin | GPS_RESET_Pin | GPS_INT_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(ALT_CS_GPIO_Port, ALT_CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : BUZZER_Pin DROGUE_Pin RECOVERY_Pin IMU_CS1_Pin
                             IMU_INT1_Pin MAG_DRDY_Pin MEM_WP_Pin MEM_HOLD_Pin
                             LED_ARMED_Pin */
    GPIO_InitStruct.Pin = BUZZER_Pin | DROGUE_Pin | MAIN_Pin | IMU_CS1_Pin | IMU_INT1_Pin | MAG_DRDY_Pin | MEM_WP_Pin | MEM_HOLD_Pin | LED_ARMED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : DROGUE_CONT_Pin */
    GPIO_InitStruct.Pin = DROGUE_CONT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DROGUE_CONT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : IMU_INT3_Pin IMU_INT4_Pin LED_FLIGHT_Pin LED_STANDBY_Pin
                             LED_STORAGE_Pin */
    GPIO_InitStruct.Pin = IMU_INT3_Pin | IMU_INT4_Pin | LED_FLIGHT_Pin | LED_STANDBY_Pin | LED_STORAGE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : RECOVERY_CONT_Pin ARM_CONT_Pin USB_VBUS_pin*/
    GPIO_InitStruct.Pin = MAIN_CONT_Pin | ARM_CONT_Pin | USB_VBUS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : IMU_CS2_Pin IMU_INT2_Pin MAG_INT_Pin MEM_CS_Pin
                             GPS_RESET_Pin GPS_INT_Pin */
    GPIO_InitStruct.Pin = IMU_CS2_Pin | IMU_INT2_Pin | MAG_INT_Pin | MEM_CS_Pin | GPS_RESET_Pin | GPS_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : ALT_CS_Pin */
    GPIO_InitStruct.Pin = ALT_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(ALT_CS_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 36000;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {
    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 65535;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
        Error_Handler();
    }
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_EXTERNAL1;
    sSlaveConfig.InputTrigger = TIM_TS_ITR1;
    if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
