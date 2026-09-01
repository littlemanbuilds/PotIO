/**
 * MIT License
 *
 * @brief Helpers and compact presets for ContinuousPot tuning.
 *
 * @file PotIO_ContinuousPotHelpers.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include "PotIO_Filters.h"
#include "PotIO_Shaping.h"
#include "PotIO_RateLimit.h"
#include "devices/PotIO_ContinuousPot.h"
#include "PotIO_Arduino.h"

namespace PotIO
{
    // ---- Config builders ---- //

    /**
     * @brief Sticky, throttle-like feel.
     *
     * @param ema_alpha EMA coefficient (0,1]. Smaller -> smoother.
     * @param soft_k Soft-zone steepness. Larger -> steeper ends.
     * @param wrap_hyst Hysteresis near 0/1 to avoid false wraps.
     * @return ContinuousPot config for a sticky knob preset.
     *
     * @note Config builders leave `reader` at its default; set it before calling setConfig().
     */
    inline auto KnobStickyConfig(float ema_alpha = 0.08f,
                                 float soft_k = 3.0f,
                                 float wrap_hyst = 0.05f)
        -> ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeSoftZone>::Config
    {
        using CP = ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeSoftZone>;
        typename CP::Config cfg;

        cfg.filter = EMAFilter{};
        cfg.filter.alpha = ema_alpha;
        cfg.shape = ShapeSoftZone{};
        cfg.shape.k = soft_k;
        cfg.rate = NoRateLimit{};
        cfg.wrap_hyst = wrap_hyst;
        return cfg;
    }

    /**
     * @brief Smooth, linear dial.
     *
     * @param ema_alpha EMA coefficient (0,1]. Smaller -> smoother.
     * @param wrap_hyst Hysteresis near 0/1 to avoid false wraps.
     * @return ContinuousPot config for a smooth linear knob preset.
     *
     * @note Config builders leave `reader` at its default; set it before calling setConfig().
     */
    inline auto KnobSmoothConfig(float ema_alpha = 0.12f,
                                 float wrap_hyst = 0.04f)
        -> ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeIdentity>::Config
    {
        using CP = ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeIdentity>;
        typename CP::Config cfg;

        cfg.filter = EMAFilter{};
        cfg.filter.alpha = ema_alpha;
        cfg.shape = ShapeIdentity{};
        cfg.rate = NoRateLimit{};
        cfg.wrap_hyst = wrap_hyst;
        return cfg;
    }

    /**
     * @brief Fast jog wheel with gentle cubic-expo.
     *
     * @param ema_alpha EMA coefficient (0,1]. Smaller -> smoother.
     * @param expo_a Cubic/expo blend factor (0..1).
     * @param wrap_hyst Hysteresis near 0/1 to avoid false wraps.
     * @return ContinuousPot config for a jog-wheel preset.
     *
     * @note Config builders leave `reader` at its default; set it before calling setConfig().
     */
    inline auto KnobJogConfig(float ema_alpha = 0.25f,
                              float expo_a = 0.65f,
                              float wrap_hyst = 0.03f)
        -> ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeCubicExpo>::Config
    {
        using CP = ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeCubicExpo>;
        typename CP::Config cfg;

        cfg.filter = EMAFilter{};
        cfg.filter.alpha = ema_alpha;
        cfg.shape = ShapeCubicExpo{};
        cfg.shape.a = expo_a;
        cfg.rate = NoRateLimit{};
        cfg.wrap_hyst = wrap_hyst;
        return cfg;
    }

    // ---- Factory helpers ---- //

    /**
     * @brief Build a sticky, throttle-like continuous knob on an Arduino ADC pin.
     *
     * @param pin ADC pin.
     * @param c Min/center/max calibration in raw ADC units.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11). Ignored on non-ESP32.
     * @param ema_alpha EMA coefficient (0,1]. Smaller -> smoother.
     * @param soft_k Soft-zone steepness.
     * @param wrap_hyst Hysteresis near 0/1 to avoid false wraps.
     * @return Configured ContinuousPot using the sticky preset.
     */
    inline ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeSoftZone>
    makeKnobSticky(uint8_t pin,
                   PotCalib c = PotCalib{},
                   int atten_db = 11,
                   float ema_alpha = 0.08f,
                   float soft_k = 3.0f,
                   float wrap_hyst = 0.05f)
    {
        setEsp32PinAttenuation(pin, atten_db);

        auto cfg = KnobStickyConfig(ema_alpha, soft_k, wrap_hyst);
        cfg.reader = ArduinoAnalogRead(pin);
        cfg.calib = c;
        return ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeSoftZone>(cfg);
    }

    /**
     * @brief Build a smooth, linear continuous knob on an Arduino ADC pin.
     *
     * @param pin ADC pin.
     * @param c Min/center/max calibration in raw ADC units.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11). Ignored on non-ESP32.
     * @param ema_alpha EMA coefficient (0,1]. Smaller -> smoother.
     * @param wrap_hyst Hysteresis near 0/1 to avoid false wraps.
     * @return Configured ContinuousPot using the smooth preset.
     */
    inline ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeIdentity>
    makeKnobSmooth(uint8_t pin,
                   PotCalib c = PotCalib{},
                   int atten_db = 11,
                   float ema_alpha = 0.12f,
                   float wrap_hyst = 0.04f)
    {
        setEsp32PinAttenuation(pin, atten_db);

        auto cfg = KnobSmoothConfig(ema_alpha, wrap_hyst);
        cfg.reader = ArduinoAnalogRead(pin);
        cfg.calib = c;
        return ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeIdentity>(cfg);
    }

    /**
     * @brief Build a fast jog-wheel style continuous knob on an Arduino ADC pin.
     *
     * @param pin ADC pin.
     * @param c Min/center/max calibration in raw ADC units.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11). Ignored on non-ESP32.
     * @param ema_alpha EMA coefficient (0,1]. Smaller -> smoother.
     * @param expo_a Cubic/expo blend factor (0..1).
     * @param wrap_hyst Hysteresis near 0/1 to avoid false wraps.
     * @return Configured ContinuousPot using the jog preset.
     */
    inline ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeCubicExpo>
    makeKnobJog(uint8_t pin,
                PotCalib c = PotCalib{},
                int atten_db = 11,
                float ema_alpha = 0.25f,
                float expo_a = 0.65f,
                float wrap_hyst = 0.03f)
    {
        setEsp32PinAttenuation(pin, atten_db);

        auto cfg = KnobJogConfig(ema_alpha, expo_a, wrap_hyst);
        cfg.reader = ArduinoAnalogRead(pin);
        cfg.calib = c;
        return ContinuousPot<ArduinoAnalogRead, EMAFilter, NoRateLimit, ShapeCubicExpo>(cfg);
    }

} ///< namespace PotIO
