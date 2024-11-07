#include "main.h"
#include "foc_lib.h"
#include "GaiasPins.h"

// Variables for FOC
float alpha, beta, dCurrent, qCurrent, theta;
float tA, tB, tC;

// Motor control variables
float throttle;
float iA, iB, iC;
uint8_t hallPosition;

// ADC channels for currents and throttle
uint16_t adcThrottle, adcIB, adcIC;

// Constants
#define THROTTLE_SCALE   (some_value)    // Define based on throttle's ADC range and desired control range
#define CURRENT_SCALE    (some_value)    // Define based on current sensor's scaling factor
#define OFFSET_ADC       (some_value)    // Define offset if needed

// Function to read Hall sensors and determine rotor position
uint8_t Read_Hall_Position(void) {
    uint8_t hallA = HAL_GPIO_ReadPin(HALLA__PORT, HALLA__PIN);
    uint8_t hallB = HAL_GPIO_ReadPin(HALLB__PORT, HALLB__PIN);
    uint8_t hallC = HAL_GPIO_ReadPin(HALLC__PORT, HALLC__PIN);

    // Combine the Hall sensor states into a 3-bit position code
    uint8_t hallState = (hallA << 2) | (hallB << 1) | hallC;

    // Convert the 3-bit position code into a rotor position sector (0 to 5)
    switch (hallState) {
        case 0b001: return 1;
        case 0b011: return 2;
        case 0b010: return 3;
        case 0b110: return 4;
        case 0b100: return 5;
        case 0b101: return 6;
        default:    return 0;  // Error state, handle appropriately
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // Read throttle and current measurements
    adcThrottle = HAL_ADC_GetValue(&hadc1); // Throttle ADC channel
    adcIB = HAL_ADC_GetValue(&hadc2);       // IB current ADC channel
    adcIC = HAL_ADC_GetValue(&hadc3);       // IC current ADC channel
    
    // Calculate phase currents
    iB = (adcIB - OFFSET_ADC) * CURRENT_SCALE;
    iC = (adcIC - OFFSET_ADC) * CURRENT_SCALE;
    iA = -(iB + iC); // Calculate iA based on Kirchhoff's law
}

void MotorControl_Loop(void) {
    // Calculate theta from hall sensors
    hallPosition = Read_Hall_Position(); // Get the hall position sector
    if (hallPosition == 0) return;       // Skip if in error state

    // Convert hall position to electrical angle (theta)
    theta = (hallPosition - 1) * (2.0f * PI / 6.0f);

    // Clarke Transform: Phase currents -> alpha-beta currents
    FOC_Clarke(iA, iB, &alpha, &beta);
    
    // Park Transform: alpha-beta currents -> D-Q currents in rotating frame
    FOC_Park(alpha, beta, sinf(theta), cosf(theta), &dCurrent, &qCurrent);
    
    // Throttle control as Q-axis reference (torque)
    float qRef = adcThrottle * THROTTLE_SCALE;

    // PID Control on D and Q currents
    PID_Type pidD, pidQ;
    FOC_PIDdefaults(&pidD);
    FOC_PIDdefaults(&pidQ);
    pidQ.Err = qRef - qCurrent;
    FOC_PIDcalc(&pidQ);
    
    // Inverse Park Transform: D-Q -> alpha-beta for SVM
    FOC_Ipark(pidD.Out, pidQ.Out, sinf(theta), cosf(theta), &alpha, &beta);
    
    // SVM Calculation for three-phase PWM generation
    FOC_SVM(alpha, beta, &tA, &tB, &tC);

    // Update PWM signals for phases A, B, C
    Set_PWM_Duty_Cycle(TIM_CHANNEL_1, tA);
    Set_PWM_Duty_Cycle(TIM_CHANNEL_2, tB);
    Set_PWM_Duty_Cycle(TIM_CHANNEL_3, tC);
}

// Main control loop
int main(void) {
    HAL_Init();
    SystemClock_Config();

    // Initialize peripherals
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_TIM1_Init(); // Assuming TIM1 for PWM generation

    while (1) {
        MotorControl_Loop();
    }
}
