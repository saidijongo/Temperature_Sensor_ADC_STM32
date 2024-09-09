/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"   // Core HAL
#include "stm32f4xx_hal_adc.h"  // ADC HAL
#include <math.h>  // For sqrt(), cos(), sin()

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// PI controller constants
#define Kp 0.05
#define Ki 0.01

// PWM and motor-related constants
#define PWM_PERIOD 1600
#define LUT_SIZE 256
uint16_t sine_lut[LUT_SIZE] = { /* Precomputed sine values from 0 to 1599 */ };

// GPIO pin definitions
#define PAS_SENSOR_PIN GPIO_PIN_5
#define PAS_SENSOR_PORT GPIOA

#define THROTTLE_ADC_PIN GPIO_PIN_0
#define THROTTLE_ADC_PORT GPIOC

#define TORQUE_SENSOR_PIN GPIO_PIN_1
#define TORQUE_SENSOR_PORT GPIOC

#define BRAKE_PIN GPIO_PIN_0
#define BRAKE_PORT GPIOB

#define HEADLIGHT_PIN GPIO_PIN_1
#define HEADLIGHT_PORT GPIOB

// Motor control related
uint32_t adc_throttle_value = 0;  // Throttle input value
uint32_t adc_torque_value = 0;    // Torque sensor value
uint32_t hall_position = 0;       // Hall sensor position
uint32_t motor_speed = 0;         // Motor speed control

/* USER CODE END PV */

/* Function prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);

/* USER CODE BEGIN PFP */
// Function prototypes for motor control
void ProcessHallSensors(void);
void UpdatePWMDutyCycle(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c);
void SinusoidalCommutation(uint32_t hall_position);
void ReadThrottle(void);

/* USER CODE END PFP */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  
  // Start PWM outputs
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  
  // Main loop
  while (1)
  {
    // Process Hall sensor inputs
    ProcessHallSensors();
    
    // Perform sinusoidal commutation based on Hall sensor position
    SinusoidalCommutation(hall_position);
    
    // Read throttle input and adjust speed
    ReadThrottle();
    
    // Control loop timing
    HAL_Delay(1);
  }
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Pin = PAS_SENSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PAS_SENSOR_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = BRAKE_PIN | HEADLIGHT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = THROTTLE_ADC_PIN | TORQUE_SENSOR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* TIM1 Initialization for PWM Generation */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = PWM_PERIOD;
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

/* ADC Initialization for Throttle and Torque */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

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
  HAL_ADC_Init(&hadc1);

  sConfig.Channel = ADC_CHANNEL_10;  // Throttle (PC0)
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_11;  // Torque Sensor (PC1)
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/* Hall Sensor Processing */
void ProcessHallSensors(void)
{
  // Simulate reading Hall sensors (example with 6-step commutation, but adjusted for sinusoidal LUT control)
  hall_position = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) << 2) |
                  (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) << 1) |
                  HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
}

/* Sinusoidal Commutation Based on Hall Sensor Position */
void SinusoidalCommutation(uint32_t hall_position)
{
  uint16_t angle = (hall_position * (LUT_SIZE / 6)) % LUT_SIZE;
  
  // Get duty cycles from the sinusoidal LUT
  uint16_t duty_a = sine_lut[angle];
  uint16_t duty_b = sine_lut[(angle + LUT_SIZE / 3) % LUT_SIZE];
  uint16_t duty_c = sine_lut[(angle + 2 * LUT_SIZE / 3) % LUT_SIZE];

  // Update the PWM duty cycles
  UpdatePWMDutyCycle(duty_a, duty_b, duty_c);
}

/* Update PWM Duty Cycle */
void UpdatePWMDutyCycle(uint16_t duty_a, uint16_t duty_b, uint16_t duty_c)
{
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_a);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_b);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_c);
}

/* Throttle Read and Update */
void ReadThrottle(void)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adc_throttle_value = HAL_ADC_GetValue(&hadc1);
}

/* Error Handler */
void Error_Handler(void)
{
  while (1)
  {
  }
}
