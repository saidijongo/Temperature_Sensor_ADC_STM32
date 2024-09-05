#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private variables */
ADC_HandleTypeDef hadc1;
UART_HandleTypeDef huart2;

double Temp1 = 0, Temp2 = 0, Temp3 = 0;
double resistance1, resistance2, resistance3;
uint16_t ADC_VAL[3];

/* Function Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART2_UART_Init(void);

void ADC_Select_CH1(void);
void ADC_Select_CH2(void);
void ADC_Select_CH3(void);

int main(void)
{
    /* MCU Initialization */
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USART2_UART_Init();

    char msg[100];  // Buffer to hold UART messages
    int resolution = 4096;  // 12-bit ADC resolution (2^12 = 4096)

    while (1)
    {
        /* Read ADC values for each channel */
        ADC_Select_CH1();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        ADC_Select_CH2();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        ADC_Select_CH3();
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 1000);
        ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        /* Calculate resistances */
        resistance1 = 10000 * ((ADC_VAL[0] / (double)resolution) / (1 - (ADC_VAL[0] / (double)resolution)));
        resistance2 = 10000 * ((ADC_VAL[1] / (double)resolution) / (1 - (ADC_VAL[1] / (double)resolution)));
        resistance3 = 10000 * ((ADC_VAL[2] / (double)resolution) / (1 - (ADC_VAL[2] / (double)resolution)));

        /* Calculate temperatures in Celsius */
        Temp1 = 1 / ((1 / 298.15) + ((double)1 / 3435) * log((double)resistance1 / 10000)) - 273.15;
        Temp2 = 1 / ((1 / 298.15) + ((double)1 / 3435) * log((double)resistance2 / 10000)) - 273.15;
        Temp3 = 1 / ((1 / 298.15) + ((double)1 / 3435) * log((double)resistance3 / 10000)) - 273.15;

        /* Prepare and send the temperature readings over UART */
        snprintf(msg, sizeof(msg), "Temp1: %.2f C, Temp2: %.2f C, Temp3: %.2f C\r\n", Temp1, Temp2, Temp3);
        HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        HAL_Delay(1000);
    }
}

/* ADC Channel Selection Functions */
void ADC_Select_CH1(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void ADC_Select_CH2(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_2;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}

void ADC_Select_CH3(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
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

/* GPIO Initialization */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
}

/* ADC1 Initialization */
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

/* System Clock Configuration */
void SystemClock_Config(void)
{
    /* Implementation of clock configuration based on your board settings */
}

/* Error Handler */
void Error_Handler(void)
{
    while (1)
    {
        // Stay here if an error occurs
    }
}
//Use A7 and A4
