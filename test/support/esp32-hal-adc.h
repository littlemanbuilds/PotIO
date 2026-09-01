#pragma once
#include <cstdint>
typedef enum
{
    ADC_0db = 0,
    ADC_2_5db = 1,
    ADC_6db = 2,
    ADC_11db = 3
} adc_attenuation_t;
inline void analogSetPinAttenuation(std::uint8_t pin, adc_attenuation_t attn)
{
    (void)pin;
    (void)attn;
}
