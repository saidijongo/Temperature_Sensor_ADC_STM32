#include "main.h"
#include "adc.h"
#include "gpio.h"

#define R_SHUNT 0.01f       // Shunt resistor value in ohms
#define ADC_MAX_VOLTAGE 3.3f // Reference voltage for ADC
#define ADC_RESOLUTION 4096  // 12-bit ADC resolution
#define OVERCURRENT_THRESHOLD 25.0f  // Overcurrent threshold in Amps

void SystemClock_Config(void);
float read_battery_current(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();

    while (1)
    {
        // Read the current through the shunt resistor
        float battery_current = read_battery_current();

        // Check if current exceeds the threshold
        if (battery_current > OVERCURRENT_THRESHOLD)
        {
            // Overcurrent detected - Take action
            // Example: Turn on an LED or disable the motor
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // Assuming LED is connected to PA5
        }
        else
        {
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }

        HAL_Delay(100); // Delay for stability
    }
}

float read_battery_current(void)
{
    uint32_t adc_value = 0;
    float shunt_voltage, battery_current;

    // Start ADC Conversion
    HAL_ADC_Start(&hadc1);

    // Wait for conversion to complete
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
    {
        // Get the ADC value
        adc_value = HAL_ADC_GetValue(&hadc1);
    }

    // Stop ADC
    HAL_ADC_Stop(&hadc1);

    // Convert ADC value to voltage
    shunt_voltage = (adc_value * ADC_MAX_VOLTAGE) / ADC_RESOLUTION;

    // Calculate current using Ohm's law (I = V / R)
    battery_current = shunt_voltage / R_SHUNT;

    return battery_current;
}
