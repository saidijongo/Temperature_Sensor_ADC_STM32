#include "main.h"
#include <math.h>
#include <stdio.h>

// Define hardware peripherals
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
GPIO_InitTypeDef PAS_Pin; // Pedal Assist Sensor GPIO
GPIO_InitTypeDef Throttle_Pin; // Throttle ADC GPIO

// Define FOC-related variables
float Ia, Ib, Ic; // Phase currents
float V_alpha, V_beta, Id, Iq, Vd, Vq, Angle, Speed_cmd;
float throttle_value;
int PAS_level;

// Motor control variables
#define POLE_PAIRS 7
#define PWM_PERIOD 1000
#define MAX_SPEED 4000
#define MIN_SPEED 0

// Declare FOC functions
void Clarke_Transform(float Ia, float Ib, float* I_alpha, float* I_beta);
void Park_Transform(float I_alpha, float I_beta, float Angle, float* Id, float* Iq);
void Inv_Park_Transform(float Vd, float Vq, float Angle, float* V_alpha, float* V_beta);
void SVPWM(float V_alpha, float V_beta);
void FOC_Controller(void);

// Initialize GPIOs for PAS sensor, throttle, and fault detection
void GPIO_Init(void);

// Initialize peripherals for PWM, ADC, and UART
void SystemClock_Config(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

// Fault handling function
void Fault_Handler(void);

// ADC callback to handle throttle and shunt readings
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    // Start PWM on TIM1
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // Start ADC for throttle and shunt current sensing
    HAL_ADC_Start_IT(&hadc1);

    while (1)
    {
        // Implement Field Oriented Control (FOC) Algorithm
        FOC_Controller(); // Run FOC control loop for each cycle

        // Check PAS sensor level
        if (HAL_GPIO_ReadPin(GPIOA, PAS_Pin.Pin) == GPIO_PIN_SET) {
            PAS_level = 1;  // Detect Pedal Assistance
        } else {
            PAS_level = 0;  // No Pedal Assistance
        }

        HAL_Delay(10);  // Small delay for stability
    }
}

/** FOC Controller **/
void FOC_Controller(void)
{
    float I_alpha, I_beta;
    
    // Clarke Transformation
    Clarke_Transform(Ia, Ib, &I_alpha, &I_beta);
    
    // Park Transformation
    Park_Transform(I_alpha, I_beta, Angle, &Id, &Iq);

    // Speed Control (Throttle and PAS Level)
    if (PAS_level == 1) {
        Speed_cmd = throttle_value * MAX_SPEED;  // Scale throttle to motor speed
    } else {
        Speed_cmd = MIN_SPEED;  // Stop motor when PAS is not detected
    }

    // FOC Speed Control Logic
    Vd = Speed_cmd;  // Regulate voltage in the d-axis (direct control)
    Vq = 0;  // Zero voltage in the q-axis (torque)

    // Inverse Park Transformation
    Inv_Park_Transform(Vd, Vq, Angle, &V_alpha, &V_beta);

    // Space Vector PWM Generation
    SVPWM(V_alpha, V_beta);
}

/** Clarke Transformation **/
void Clarke_Transform(float Ia, float Ib, float* I_alpha, float* I_beta)
{
    *I_alpha = Ia;
    *I_beta = (1.0 / sqrt(3)) * (Ia + 2 * Ib);
}

/** Park Transformation **/
void Park_Transform(float I_alpha, float I_beta, float Angle, float* Id, float* Iq)
{
    *Id = I_alpha * cos(Angle) + I_beta * sin(Angle);
    *Iq = -I_alpha * sin(Angle) + I_beta * cos(Angle);
}

/** Inverse Park Transformation **/
void Inv_Park_Transform(float Vd, float Vq, float Angle, float* V_alpha, float* V_beta)
{
    *V_alpha = Vd * cos(Angle) - Vq * sin(Angle);
    *V_beta = Vd * sin(Angle) + Vq * cos(Angle);
}

/** Space Vector PWM (SVPWM) **/
void SVPWM(float V_alpha, float V_beta)
{
    // Calculate time durations for each sector
    float T1, T2, T0;
    int sector;
    
    // Determine the sector
    if (V_beta > 0) {
        sector = (V_alpha >= 0) ? 1 : 2;
    } else {
        sector = (V_alpha >= 0) ? 6 : 5;
    }

    // Calculate time T1, T2, and T0 based on V_alpha, V_beta and sector

    // Set the PWM duty cycle based on calculated time durations
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, T1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, T2);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, T0);
}

/** ADC Callback - Handle throttle input and shunt current sensing **/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Throttle input from ADC channel
    throttle_value = HAL_ADC_GetValue(hadc);  // Read throttle potentiometer value
    throttle_value = throttle_value / 4096.0;  // Normalize between 0 and 1

    // Read phase currents from shunt amplifiers
    Ia = HAL_ADC_GetValue(&hadc1);  // Phase A current
    Ib = HAL_ADC_GetValue(&hadc1);  // Phase B current
    Ic = -Ia - Ib;  // Kirchhoff's law (assuming balanced system)
}

/** GPIO Initialization **/
void GPIO_Init(void)
{
    // PAS Pin Initialization
    __HAL_RCC_GPIOA_CLK_ENABLE();
    PAS_Pin.Pin = GPIO_PIN_0;
    PAS_Pin.Mode = GPIO_MODE_INPUT;
    PAS_Pin.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &PAS_Pin);

    // Throttle Pin Initialization
    Throttle_Pin.Pin = GPIO_PIN_1;
    Throttle_Pin.Mode = GPIO_MODE_ANALOG;
    Throttle_Pin.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &Throttle_Pin);
}

/** PWM Initialization **/
void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = PWM_PERIOD;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);

    // Configure PWM channels
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
}

/** ADC Initialization **/
void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 3;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    HAL_ADC_Init(&hadc1);

    // Configure ADC Channels for Phase Currents and Throttle
    sConfig.Channel = ADC_CHANNEL_0;  // Phase A Current
    sConfig.Rank = 1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_1;  // Phase B Current
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    sConfig.Channel = ADC_CHANNEL_2;  // Throttle Input
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/** UART Initialization **/
void MX_USART2_UART_Init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();

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

/** System Clock Configuration **/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/** Fault Handler **/
void Fault_Handler(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // Turn on Fault LED
    while(1) {
        // Enter fault condition
    }
}
