#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

double Temp[3] = {0};  // Array to hold temperatures
double resistance[3] = {0};  // Array to hold resistances
uint16_t ADC_VAL[3];  // ADC values array

char uart_rx_buffer[10];  // Buffer for receiving UART commands
char uart_tx_buffer[100];  // Buffer for sending UART messages

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);
void ADC_Select_Channel(uint32_t channel);
void Process_UART_Command(void);
void Read_ADC_Values(void);
void Calculate_Resistances(void);
void Calculate_Temperatures(void);
void Send_Temperatures(void);

int main(void)
{
    /* MCU Initialization */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    /* Infinite loop */
    while (1)
    {
        Read_ADC_Values();  // Read all ADC channels
        Calculate_Resistances();  // Convert ADC values to resistances
        Calculate_Temperatures();  // Convert resistances to temperatures
        
        // Check if any UART data is received and process it
        if (HAL_UART_Receive(&huart2, (uint8_t*)uart_rx_buffer, sizeof(uart_rx_buffer), 10) == HAL_OK)
        {
            Process_UART_Command();
        }

        // Send the temperatures via UART every second
        Send_Temperatures();
        HAL_Delay(1000);
    }
}

/* ADC Channel Selection Function (Refactored with Loop) */
void ADC_Select_Channel(uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Function to read ADC values from 3 channels */
void Read_ADC_Values(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        ADC_Select_Channel(ADC_CHANNEL_1 + i);  // Select channel 1, 2, or 3
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        ADC_VAL[i] = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);
    }
}

/* Calculate resistances from ADC values */
void Calculate_Resistances(void)
{
    int resolution = 4096;
    for (uint8_t i = 0; i < 3; i++)
    {
        resistance[i] = 10000 * ((ADC_VAL[i] / (double)resolution) / (1 - (ADC_VAL[i] / (double)resolution)));
    }
}

/* Calculate temperatures from resistances */
void Calculate_Temperatures(void)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        Temp[i] = 1 / ((1 / 298.15) + (1 / 3435.0) * log(resistance[i] / 10000));
        Temp[i] -= 273.15;  // Convert from Kelvin to Celsius
    }
}

/* Send the temperatures over UART */
void Send_Temperatures(void)
{
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "Temp1: %.2f C, Temp2: %.2f C, Temp3: %.2f C\r\n", Temp[0], Temp[1], Temp[2]);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), HAL_MAX_DELAY);
}

/* Process UART command */
void Process_UART_Command(void)
{
    if (strcmp(uart_rx_buffer, "GET") == 0)
    {
        Send_Temperatures();  // Send temperatures if command is "GET"
    }
    else if (strcmp(uart_rx_buffer, "RESET") == 0)
    {
        HAL_NVIC_SystemReset();  // Reset the MCU if command is "RESET"
    }
    // Clear the UART buffer after processing
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
}

/* UART Initialization */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ADC Initialization */
static void MX_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LD2_Pin */
    GPIO_InitStruct.Pin = LD2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 180;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_PWREx_EnableOverDrive() != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}
