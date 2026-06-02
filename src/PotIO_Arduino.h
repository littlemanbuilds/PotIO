/**
 * MIT License
 *
 * @brief Arduino platform readers and helpers (analog read, ESP32 attenuation).
 *
 * @file PotIO_Arduino.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include <PotIO_Compatibility.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <esp32-hal-adc.h>
#endif

namespace PotIO
{
    /**
     * @brief Reader adapter around Arduino analogRead().
     *
     * @details
     * On Arduino cores where analogReadResolution() can be configured, the default full-scale
     * may not match your runtime resolution. For best results:
     *  - define POTIO_ARDUINO_ADC_FULLSCALE, or
     *  - create your own Reader with a correct `kFullScale`.
     */
    struct ArduinoAnalogRead
    {
        static constexpr int kFullScale = POTIO_ARDUINO_ADC_FULLSCALE; ///< Default full-scale in raw units.

        uint8_t pin{0}; ///< ADC pin (Arduino numbering).

        /**
         * @brief Construct an Arduino analog reader for one pin.
         * @param p ADC pin in Arduino numbering.
         */
        explicit ArduinoAnalogRead(uint8_t p = 0) noexcept : pin(p) {}

        /**
         * @brief Perform a raw ADC read.
         * @return int Raw ADC value.
         */
        int operator()() const noexcept
        {
#if defined(ARDUINO)
            return ::analogRead(pin);
#else
            return 0;
#endif
        }
    };

#if defined(ARDUINO_ARCH_ESP32)
    /**
     * @brief ESP32 reader adapter returning millivolts via analogReadMilliVolts().
     *
     * @details
     * This adapter is intentionally ESP32-only because analogReadMilliVolts()
     * is provided by the ESP32 Arduino core, not by generic Arduino cores.
     *
     * @tparam FullScaleMv Full-scale in mV (used as kFullScale for calibration).
     */
    template <int FullScaleMv = 3300>
    struct ArduinoAnalogReadMilliVolts
    {
        static constexpr int kFullScale = FullScaleMv; ///< Full-scale in mV.
        uint8_t pin{0};                                ///< ADC pin (Arduino numbering).

        /**
         * @brief Construct an ESP32 millivolt reader for one pin.
         * @param p ADC pin in Arduino numbering.
         */
        explicit ArduinoAnalogReadMilliVolts(uint8_t p = 0) noexcept : pin(p) {}

        /**
         * @brief Perform an ADC read in millivolts.
         * @return int ADC value in mV.
         */
        int operator()() const noexcept
        {
            return ::analogReadMilliVolts(pin);
        }
    };
#endif

    /**
     * @brief Set ESP32 ADC pin attenuation (Arduino core; no-op elsewhere).
     * @param pin ADC pin.
     * @param attn_db Attenuation in dB (0, 3, 6, 11 typical).
     */
    inline void setEsp32PinAttenuation(uint8_t pin, int attn_db) noexcept
    {
#if defined(ARDUINO_ARCH_ESP32)
#ifndef ADC_0db
#define ADC_0db ((adc_attenuation_t)0)
#endif
#ifndef ADC_2_5db
#define ADC_2_5db ((adc_attenuation_t)1)
#endif
#ifndef ADC_6db
#define ADC_6db ((adc_attenuation_t)2)
#endif
#ifndef ADC_11db
#define ADC_11db ((adc_attenuation_t)3)
#endif
        const adc_attenuation_t attn =
            (attn_db >= 11) ? ADC_11db : (attn_db >= 6) ? ADC_6db
                                 : (attn_db >= 3)   ? ADC_2_5db
                                                    : ADC_0db;

        ::analogSetPinAttenuation(pin, attn);
#else
        (void)pin;
        (void)attn_db;
#endif
    }

} ///< namespace PotIO
