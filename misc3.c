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
#include "stm32f4xx_hal.h"   // Core HAL
#include "stm32f4xx_hal_adc.h"  // ADC HAL
#include <math.h>  // For sqrt(), cos(), sin()

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// PI controller constants
#define Kp 0.05
#define Ki 0.01

// GPIO pin definitions
#define PAS_SENSOR_PIN GPIO_PIN_5   // PAS sensor on PA5
#define PAS_SENSOR_PORT GPIOA

#define THROTTLE_ADC_PIN GPIO_PIN_0  // Throttle ADC on PC0 (ADC1_IN10)
#define THROTTLE_ADC_PORT GPIOC

#define TORQUE_SENSOR_PIN GPIO_PIN_1  // Torque sensor ADC on PC1 (ADC1_IN11)
#define TORQUE_SENSOR_PORT GPIOC

#define BRAKE_PIN GPIO_PIN_0  // Brake input on PB0
#define BRAKE_PORT GPIOB

#define HEADLIGHT_PIN GPIO_PIN_1  // Headlight control on PB1
#define HEADLIGHT_PORT GPIOB

// PWM for motor control (gate driver) - TIM1 on PA8, PA9, PA10
#define PWM_TIM1_CHANNEL1_PIN GPIO_PIN_8   // PA8 TIM1 Channel 1
#define PWM_TIM1_CHANNEL2_PIN GPIO_PIN_9   // PA9 TIM1 Channel 2
#define PWM_TIM1_CHANNEL3_PIN GPIO_PIN_10  // PA10 TIM1 Channel 3
#define PWM_PORT GPIOA

// Hall sensors (TIM3) on PC6, PC7, and PC8
#define HALL_SENSOR_PIN1 GPIO_PIN_6  // Hall sensor 1 on PC6 (TIM3 CH1)
#define HALL_SENSOR_PIN2 GPIO_PIN_7  // Hall sensor 2 on PC7 (TIM3 CH2)
#define HALL_SENSOR_PIN3 GPIO_PIN_8  // Hall sensor 3 on PC8 (TIM3 CH3)
#define HALL_SENSOR_PORT GPIOC

// UART for LCD (S866) - USART1 on PA9 (TX) and PA10 (RX)
#define LCD_TX_PIN GPIO_PIN_6   // PA9 USART1 TX
#define LCD_RX_PIN GPIO_PIN_7  // PA10 USART1 RX
#define LCD_PORT GPIOB

// UART for debugging - USART2 on PA2 (TX) and PA3 (RX)
#define DEBUG_TX_PIN GPIO_PIN_2   // PA2 USART2 TX
#define DEBUG_RX_PIN GPIO_PIN_3   // PA3 USART2 RX
#define DEBUG_PORT GPIOA

// Motor-related variables
float I_alpha, I_beta, I_d, I_q;  // Phase currents
float V_alpha, V_beta;  // Phase voltages
float theta_e = 0;  // Electrical angle
float speed_command = 0;  // Desired speed
float torque_command = 0;  // Torque command
uint32_t adc_shunt_values[3];  // Current sensing
uint32_t adc_throttle_value;  // Throttle ADC value
uint32_t adc_torque_sensor_value;  // Torque sensor ADC value
uint32_t cadence_sensor_count = 0;  // Cadence sensor count
uint8_t PAS_level = 1;  // PAS level (1-5)
uint8_t drv_fault = 0;  // Fault status
float Iq_integral = 0;  // Integral term for PI controller

// S866 LCD Communication buffer
uint8_t RxBuffer[32];  // Receiving data from LCD
uint8_t TxBuffer[32];  // Sending data to LCD

// LCD Protocol (based on manual file)
#define S866_PROTOCOL_START 0x55

// Other constants
#define MAX_THROTTLE 4096
#define MOTOR_MAX_SPEED 1000
#define MAX_TORQUE_SENSOR_VALUE 1000
#define MAX_CADENCE_SENSOR_VALUE 12
#define ADC_GAIN 40
#define PI 3.14159265359

/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
void Check_DRV8353_Faults(void);

/* USER CODE BEGIN PFP */
// Function prototypes
void Process_FOC_Controller(void);
void PWM_Output_Update(float v_alpha, float v_beta, float theta);
void Clarke_Transform(float Ia, float Ib, float *Ialpha, float *Ibeta);
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq);
void Inverse_Park_Transform(float Vd, float Vq, float theta, float *Valpha, float *Vbeta);
void Process_ADC(void);
void Update_Throttle(void);
void Handle_PAS_Sensor(void);
void Handle_Torque_Sensor(void);
void Calculate_PI_Control(float desired_current, float actual_current);
void Handle_LCD_Communication(void);
void Handle_Brake_Light(void);
void Handle_Headlight(void);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  // Start PWM and ADC operations
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_shunt_values, 3);

  /* USER CODE END 1 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Process ADC readings from shunt resistors, throttle, and torque sensor
    Process_ADC();

    // Handle PAS sensor (cadence sensor) and torque sensor
    Handle_PAS_Sensor();
    Handle_Torque_Sensor();

    // Update throttle input from potentiometer
    Update_Throttle();

    // Run the FOC control algorithm
    Process_FOC_Controller();

    // Check and handle faults from DRV8353
    Check_DRV8353_Faults();

    // Handle UART communication with LCD (S866)
    Handle_LCD_Communication();

    // Handle braking and brake light
    Handle_Brake_Light();

    // Handle headlight control
    Handle_Headlight();

    HAL_Delay(1);  // Control loop timing
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
}

/* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  // GPIO Initialization Code Here
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable the GPIO Clocks
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PAS Sensor Initialization (PA5)
  GPIO_InitStruct.Pin = PAS_SENSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PAS_SENSOR_PORT, &GPIO_InitStruct);

  // Brake Light GPIO Initialization (PB0)
  GPIO_InitStruct.Pin = BRAKE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BRAKE_PORT, &GPIO_InitStruct);

  // Headlight GPIO Initialization (PB1)
  GPIO_InitStruct.Pin = HEADLIGHT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HEADLIGHT_PORT, &GPIO_InitStruct);

  // Throttle (PC0) and Torque Sensor (PC1)
  GPIO_InitStruct.Pin = THROTTLE_ADC_PIN | TORQUE_SENSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  TIM_IC_InitTypeDef sConfigIC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim3);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);  // PC6
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);  // PC7
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);  // PC8
}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim8);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3);

  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart1);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

/**
  * @brief ADC1 Initialization Function (for PC0 and PC1)
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  // ADC1 configuration
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure for the selected ADC regular channels
  sConfig.Channel = ADC_CHANNEL_10;  // PC0 (Throttle)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_11;  // PC1 (Torque sensor)
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function (for PA0 and PA4)
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  // ADC2 configuration
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure for the selected ADC regular channels
  sConfig.Channel = ADC_CHANNEL_0;  // PA0-WKUP (ADC2_IN0)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_4;  // PA4 (ADC2_IN4)
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC3 Initialization Function (for PA1)
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  // ADC3 configuration
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  // Configure for the selected ADC regular channel
  sConfig.Channel = ADC_CHANNEL_1;  // PA1 (ADC3_IN1)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Check and handle faults from DRV8353 motor driver.
  */
void Check_DRV8353_Faults(void)
{
    // Assuming the DRV8353 fault pin is connected to PB0
    if (HAL_GPIO_ReadPin(BRAKE_PORT, BRAKE_PIN) == GPIO_PIN_RESET) // Assuming active low fault signal
    {
        drv_fault = 1;
        // Stop the motor in case of a fault
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        // Log the error or handle the fault here (e.g., reset the driver)
    }
    else
    {
        drv_fault = 0;  // No fault detected
    }
}


/* USER CODE BEGIN 4 */

// ADC processing function
void Process_ADC(void)
{
    // Read throttle ADC value (PC0 - ADC1)
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_throttle_value = HAL_ADC_GetValue(&hadc1);

    // Read torque sensor ADC value (PC1 - ADC1)
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_torque_sensor_value = HAL_ADC_GetValue(&hadc1);
}

// Throttle input update function
void Update_Throttle(void)
{
    speed_command = (float)adc_throttle_value / MAX_THROTTLE;  // Normalize throttle input (0 to 1.0)
}

// PAS sensor handling function
void Handle_PAS_Sensor(void)
{
    // Example: PAS sensor on PA5
    if (HAL_GPIO_ReadPin(PAS_SENSOR_PORT, PAS_SENSOR_PIN) == GPIO_PIN_SET)
    {
        cadence_sensor_count++;
        if (cadence_sensor_count > MAX_CADENCE_SENSOR_VALUE)
        {
            cadence_sensor_count = 0;  // Reset after one full revolution
        }
    }

    // Calculate torque command based on cadence sensor and PAS level
    float cadence_factor = (float)cadence_sensor_count / MAX_CADENCE_SENSOR_VALUE;
    float assist_factor = (float)PAS_level / 5.0;  // PAS level from 1 to 5
    torque_command = assist_factor * cadence_factor * MOTOR_MAX_SPEED;
}

// Torque sensor handling function
void Handle_Torque_Sensor(void)
{
    float torque_factor = (float)adc_torque_sensor_value / MAX_TORQUE_SENSOR_VALUE;
    torque_command += torque_factor * PAS_level * MOTOR_MAX_SPEED;  // Adjust with torque sensor
}

// PI controller for FOC
void Calculate_PI_Control(float desired_current, float actual_current)
{
    float error = desired_current - actual_current;
    Iq_integral += error * Ki;  // Integral term
    I_q = Kp * error + Iq_integral;  // PI control output
}

// FOC processing function
void Process_FOC_Controller(void)
{
    // Clarke and Park transforms
    Clarke_Transform(adc_shunt_values[0] / ADC_GAIN, adc_shunt_values[1] / ADC_GAIN, &I_alpha, &I_beta);
    Park_Transform(I_alpha, I_beta, theta_e, &I_d, &I_q);

    // PI control for current
    Calculate_PI_Control(speed_command, I_q);
    I_d = 0;  // Zero d-axis current for simplicity

    // Inverse Park transform
    Inverse_Park_Transform(I_d, I_q, theta_e, &V_alpha, &V_beta);

    // Update PWM outputs
    PWM_Output_Update(V_alpha, V_beta, theta_e);

    // Increment theta (simulate rotor position)
    theta_e += 0.01;
    if (theta_e > 2 * PI) theta_e -= 2 * PI;
}

// PWM update function
void PWM_Output_Update(float v_alpha, float v_beta, float theta)
{
    // Compute PWM duty cycles based on alpha-beta voltages
    float duty_a = v_alpha;
    float duty_b = -0.5 * v_alpha + sqrt(3.0) / 2 * v_beta;
    float duty_c = -0.5 * v_alpha - sqrt(3.0) / 2 * v_beta;

    // Set PWM duty cycles for TIM1 (high-side driver)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_a * 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_b * 1000);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_c * 1000);
}

// UART communication with S866 LCD
void Handle_LCD_Communication(void)
{
    // Implement UART communication based on protocol in the S866 manual
    uint8_t requestBatteryCmd[] = {
        S866_PROTOCOL_START,
        0x03,  // Request command
        0x01,  // Battery level
        0x00   // Checksum placeholder
    };
    requestBatteryCmd[3] = CalculateChecksum(requestBatteryCmd, 3);  // Calculate checksum

    // Send the command
    HAL_UART_Transmit(&huart1, requestBatteryCmd, sizeof(requestBatteryCmd), HAL_MAX_DELAY);

    // Receive the response
    HAL_UART_Receive(&huart1, RxBuffer, sizeof(RxBuffer), HAL_MAX_DELAY);
}

// Brake light control
void Handle_Brake_Light(void)
{
    if (HAL_GPIO_ReadPin(BRAKE_PORT, BRAKE_PIN) == GPIO_PIN_SET)
    {
        // Activate brake light (Example: Use PB0 as an output pin for brake light)
        HAL_GPIO_WritePin(BRAKE_PORT, BRAKE_PIN, GPIO_PIN_SET);
    }
}

// Headlight control
void Handle_Headlight(void)
{
    if (HAL_GPIO_ReadPin(HEADLIGHT_PORT, HEADLIGHT_PIN) == GPIO_PIN_SET)
    {
        // Activate headlight (Example: Use PB1 as an output pin for headlight)
        HAL_GPIO_WritePin(HEADLIGHT_PORT, HEADLIGHT_PIN, GPIO_PIN_SET);
    }
}

// Clarke Transformation
void Clarke_Transform(float Ia, float Ib, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (Ia + 2 * Ib) / sqrt(3.0);
}

// Park Transformation
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq)
{
    *Id = Ialpha * cos(theta) + Ibeta * sin(theta);
    *Iq = -Ialpha * sin(theta) + Ibeta * cos(theta);
}

// Inverse Park Transformation
void Inverse_Park_Transform(float Vd, float Vq, float theta, float *Valpha, float *Vbeta)
{
    *Valpha = Vd * cos(theta) - Vq * sin(theta);
    *Vbeta = Vd * sin(theta) + Vq * cos(theta);
}

// Checksum Calculation
uint8_t CalculateChecksum(uint8_t *data, uint16_t length)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++)
    {
        checksum += data[i];
    }
    return checksum;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
