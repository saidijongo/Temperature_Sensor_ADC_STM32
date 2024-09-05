#include "main.h"
#include "math.h"
#include "stm32f4xx_hal.h"

// Define hardware peripherals
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;
GPIO_InitTypeDef PAS_sensor_GPIO, Torque_sensor_GPIO; // PAS and Torque sensors

// Constants for motor, sensors, and PI control
#define POLE_PAIRS 7
#define MAX_THROTTLE 4096
#define MOTOR_MAX_SPEED 1000 // Max RPM
#define MAX_TORQUE_SENSOR_VALUE 1000 // Max torque sensor output
#define MAX_CADENCE_SENSOR_VALUE 12 // Maximum cadence (12 magnets)
#define ADC_GAIN 40 // Amplifier gain of DRV8353RS amplifiers
#define PI 3.14159265359

// PI controller constants
#define Kp 0.05  // Proportional gain for current
#define Ki 0.01  // Integral gain for current

// Variables for FOC control
float I_alpha, I_beta, I_d, I_q; // Phase currents
float V_alpha, V_beta; // Voltages in Clarke space
float theta_e = 0; // Electrical angle
float speed_command = 0; // Speed control from throttle (0 to 1.0)
float torque_command = 0; // Torque control (based on torque sensor)
uint32_t adc_shunt_values[3]; // ADC readings from shunt resistors
uint32_t adc_throttle_value; // ADC value from throttle
uint32_t adc_torque_sensor_value; // ADC value from torque sensor
uint32_t cadence_sensor_count = 0; // Cadence sensor (magnet count)

// PAS power assist levels
uint8_t PAS_level = 1; // Default PAS level

// DRV8353 Fault Status
uint8_t drv_fault = 0;

// PI Controller variables
float Iq_integral = 0; // Integral term for PI controller

// Function Prototypes
void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_TIM1_Init(void);
void MX_ADC1_Init(void);
void MX_USART2_UART_Init(void);
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
void Check_DRV8353_Faults(void);

// Main function
int main(void)
{
    // HAL Initialization and system configuration
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();
    
    // Start PWM on TIM1 for motor control
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    
    // Start ADC for shunt current sensing, throttle, and torque sensor
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_shunt_values, 3);
    HAL_ADC_Start_IT(&hadc1);

    // Main loop
    while (1)
    {
        // Process the ADC values from shunts and throttle
        Process_ADC();
        
        // Handle PAS sensor and adjust motor power assist level
        Handle_PAS_Sensor();
        
        // Handle torque sensor and adjust motor torque command
        Handle_Torque_Sensor();
        
        // Update throttle input from potentiometer
        Update_Throttle();
        
        // Run FOC control algorithm
        Process_FOC_Controller();
        
        // Check and handle any faults from DRV8353
        Check_DRV8353_Faults();
    }
}

// Process the FOC Controller
void Process_FOC_Controller(void)
{
    // Clarke transformation (3-phase to 2-phase)
    Clarke_Transform(adc_shunt_values[0] / ADC_GAIN, adc_shunt_values[1] / ADC_GAIN, &I_alpha, &I_beta);
    
    // Park transformation (convert 2-phase to rotating d/q frame)
    Park_Transform(I_alpha, I_beta, theta_e, &I_d, &I_q);
    
    // Speed and torque control (with PI controller)
    Calculate_PI_Control(speed_command, I_q); // Control q-axis current for speed
    I_d = 0; // No direct axis current (for simplicity)
    
    // Inverse Park transformation (convert back to alpha-beta)
    Inverse_Park_Transform(I_d, I_q, theta_e, &V_alpha, &V_beta);
    
    // Update PWM duty cycle based on alpha-beta voltages
    PWM_Output_Update(V_alpha, V_beta, theta_e);
    
    // Increment electrical angle for the next iteration (simulate motor rotation)
    theta_e += 0.01; // This needs to be linked to actual motor speed
    if (theta_e > 2 * PI) theta_e -= 2 * PI;
}

// Clarke Transformation (3-phase to alpha-beta)
void Clarke_Transform(float Ia, float Ib, float *Ialpha, float *Ibeta)
{
    *Ialpha = Ia;
    *Ibeta = (Ia + 2 * Ib) / sqrt(3.0);
}

// Park Transformation (alpha-beta to d-q frame)
void Park_Transform(float Ialpha, float Ibeta, float theta, float *Id, float *Iq)
{
    *Id = Ialpha * cos(theta) + Ibeta * sin(theta);
    *Iq = -Ialpha * sin(theta) + Ibeta * cos(theta);
}

// Inverse Park Transformation (d-q to alpha-beta)
void Inverse_Park_Transform(float Vd, float Vq, float theta, float *Valpha, float *Vbeta)
{
    *Valpha = Vd * cos(theta) - Vq * sin(theta);
    *Vbeta = Vd * sin(theta) + Vq * cos(theta);
}

// PWM Output Update (based on alpha-beta voltages)
void PWM_Output_Update(float v_alpha, float v_beta, float theta)
{
    // Example logic for PWM generation based on Clarke space voltages
    float duty_a = v_alpha;
    float duty_b = -0.5 * v_alpha + sqrt(3.0) / 2 * v_beta;
    float duty_c = -0.5 * v_alpha - sqrt(3.0) / 2 * v_beta;
    
    // Apply duty cycles to PWM (using TIM1 PWM channels)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_a * 1000); // Phase A
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_b * 1000); // Phase B
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty_c * 1000); // Phase C
}

// ADC Processing (get shunt currents, throttle, and torque sensor values)
void Process_ADC(void)
{
    // Get current readings from shunt resistors (IA, IB, IC) and throttle
    Ia = adc_shunt_values[0] / ADC_GAIN;
    Ib = adc_shunt_values[1] / ADC_GAIN;
    Ic = adc_shunt_values[2] / ADC_GAIN;
    
    // Read throttle potentiometer
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    adc_throttle_value = HAL_ADC_GetValue(&hadc1);
}

// Throttle control (use ADC value to set speed command)
void Update_Throttle(void)
{
    speed_command = adc_throttle_value / MAX_THROTTLE; // Normalize throttle input (0 to 1.0)
}

// Handle PAS Sensor (cadence sensor with 12 magnets, 5 levels)
void Handle_PAS_Sensor(void)
{
    // Simulate counting magnets in cadence sensor (12 magnets in total)
    if (HAL_GPIO_ReadPin(PAS_sensor_GPIO.Pin, PAS_sensor_GPIO.Pin) == GPIO_PIN_SET)
    {
        cadence_sensor_count++;
        if (cadence_sensor_count > MAX_CADENCE_SENSOR_VALUE)
        {
            cadence_sensor_count = 0; // Reset after a full revolution
        }
    }
    
    // Determine motor power assist based on cadence and PAS level
    float cadence_factor = (float)cadence_sensor_count / MAX_CADENCE_SENSOR_VALUE;
    float assist_factor = (float)PAS_level / 5.0; // Level 1 to 5
    torque_command = assist_factor * cadence_factor * MOTOR_MAX_SPEED;
}

// Handle Torque Sensor (pedal force sensor)
void Handle_Torque_Sensor(void)
{
    // Read ADC value from torque sensor
    adc_torque_sensor_value = HAL_ADC_GetValue(&hadc1);
    
    // Calculate torque-based motor power assist
    float torque_factor = (float)adc_torque_sensor_value / MAX_TORQUE_SENSOR_VALUE;
    torque_command += torque_factor * PAS_level * MOTOR_MAX_SPEED; // Adjust command with torque
}

// PI Controller for regulating motor current
void Calculate_PI_Control(float desired_current, float actual_current)
{
    float error = desired_current - actual_current;
    Iq_integral += error * Ki; // Integral term
    I_q = Kp * error + Iq_integral; // Proportional + Integral control
}

// Check for DRV8353 faults and handle them
void Check_DRV8353_Faults(void)
{
    // Check for faults (e.g., overcurrent, undervoltage)
    if (HAL_GPIO_ReadPin(GPIO_PIN_DRV_FAULT, GPIO_PIN_DRV_FAULT) == GPIO_PIN_RESET)
    {
        // Fault detected, stop the motor
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
        
        // Handle specific faults here (log error, reset driver, etc.)
    }
}

// Initialize GPIO for PAS sensor and fault detection
void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    PAS_sensor_GPIO.Pin = GPIO_PIN_5; // Example GPIO for PAS sensor
    PAS_sensor_GPIO.Mode = GPIO_MODE_IT_FALLING;
    PAS_sensor_GPIO.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &PAS_sensor_GPIO);
    
    // Setup GPIO for DRV8353 fault pin (input)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_DRV_FAULT;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// Initialize TIM1 for PWM generation
void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 1000; // PWM period
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim1);
    
    // Setup PWM mode for channels
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0; // Initial duty cycle
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
}

// Initialize ADC for shunt current sensing, throttle, and torque input
void MX_ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();
    
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 4;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    HAL_ADC_Init(&hadc1);
    
    // Configure ADC channels for current shunts and throttle
    sConfig.Channel = ADC_CHANNEL_0; // Shunt current phase A
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_1; // Shunt current phase B
    sConfig.Rank = 2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_2; // Shunt current phase C
    sConfig.Rank = 3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
    
    sConfig.Channel = ADC_CHANNEL_3; // Throttle (potentiometer)
    sConfig.Rank = 4;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

// Initialize UART for debugging
void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

// System Clock Configuration
void SystemClock_Config(void)
{
    // Configure the system clock as required
}

// Error Handler
void Error_Handler(void)
{
    while (1) {}
}
