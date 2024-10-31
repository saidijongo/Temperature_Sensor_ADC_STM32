uint32_t VR[2];

void Read_ADC_Channels() {
    // Channel 1
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        VR[0] = HAL_ADC_GetValue(&hadc1); // Store result for Channel 1
    }
    HAL_ADC_Stop(&hadc1);

    // Channel 2
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        VR[1] = HAL_ADC_GetValue(&hadc1); // Store result for Channel 2
    }
    HAL_ADC_Stop(&hadc1);
}
