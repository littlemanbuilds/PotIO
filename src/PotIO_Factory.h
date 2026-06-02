/**
 * MIT License
 *
 * @brief Factory helpers for constructing common PotIO devices on Arduino.
 *
 * @file PotIO_Factory.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include <PotIO_Arduino.h>
#include <devices/PotIO_LinearPot.h>
#include <devices/PotIO_ContinuousPot.h>
#include <devices/PotIO_Joystick2D.h>
#include <devices/PotIO_SteppedPot.h>

namespace PotIO
{
    // ---- LinearPot factories ---- //

    /**
     * @brief Helper: create a LinearPot using raw ADC units (analogRead) with optional filtering/shaping.
     *
     * @tparam Filter Filter type.
     * @tparam Rate Rate limiter type.
     * @tparam Shaper Shaper type.
     *
     * @param pin ADC pin.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11). Ignored on non-ESP32.
     * @param f Filter instance.
     * @param rate Rate limiter instance.
     * @param s Shaper instance.
     * @return Configured LinearPot using ArduinoAnalogRead.
     */
    template <typename Filter = EMAFilter, typename Rate = NoRateLimit, typename Shaper = ShapeIdentity>
    inline LinearPot<ArduinoAnalogRead, Filter, Rate, Shaper>
    makePotAnalog(uint8_t pin, int atten_db = 11, Filter f = Filter{}, Rate rate = Rate{}, Shaper s = Shaper{})
    {
        setEsp32PinAttenuation(pin, atten_db);

        using Reader = ArduinoAnalogRead;
        typename LinearPot<Reader, Filter, Rate, Shaper>::Config cfg;
        cfg.reader = Reader(pin);
        cfg.filter = f;
        cfg.rate = rate;
        cfg.shape = s;
        return LinearPot<Reader, Filter, Rate, Shaper>(cfg);
    }

    /**
     * @brief Helper: create a LinearPot using raw ADC units with centered calibration.
     *
     * @tparam Filter Filter type.
     * @tparam Rate Rate limiter type.
     * @tparam Shaper Shaper type.
     *
     * @param pin ADC pin.
     * @param c Min/center/max calibration in raw ADC units.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11). Ignored on non-ESP32.
     * @param f Filter instance.
     * @param rate Rate limiter instance.
     * @param s Shaper instance.
     * @return Configured LinearPot using ArduinoAnalogRead.
     */
    template <typename Filter = EMAFilter, typename Rate = NoRateLimit, typename Shaper = ShapeIdentity>
    inline LinearPot<ArduinoAnalogRead, Filter, Rate, Shaper>
    makePotAnalog(uint8_t pin, PotCalib c, int atten_db = 11, Filter f = Filter{}, Rate rate = Rate{}, Shaper s = Shaper{})
    {
        setEsp32PinAttenuation(pin, atten_db);

        using Reader = ArduinoAnalogRead;
        typename LinearPot<Reader, Filter, Rate, Shaper>::Config cfg;
        cfg.reader = Reader(pin);
        cfg.calib = c;
        cfg.filter = f;
        cfg.rate = rate;
        cfg.shape = s;
        return LinearPot<Reader, Filter, Rate, Shaper>(cfg);
    }

#if defined(ARDUINO_ARCH_ESP32)
    /**
     * @brief ESP32 helper: create a LinearPot reading millivolts via analogReadMilliVolts().
     *
     * @details
     * This is useful on ESP32 where the raw ADC value can vary by attenuation/resolution.
     * Provide a FullScaleMv that matches your expected range (e.g., 3300).
     *
     * @tparam FullScaleMv Full-scale in millivolts used for normalization.
     * @tparam Filter Filter type.
     * @tparam Rate Rate limiter type.
     * @tparam Shaper Shaper type.
     *
     * @param pin ADC pin.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11).
     * @param f Filter instance.
     * @param rate Rate limiter instance.
     * @param s Shaper instance.
     * @return Configured ESP32 millivolt LinearPot.
     */
    template <int FullScaleMv = 3300, typename Filter = EMAFilter, typename Rate = NoRateLimit, typename Shaper = ShapeIdentity>
    inline LinearPot<ArduinoAnalogReadMilliVolts<FullScaleMv>, Filter, Rate, Shaper>
    makePotMilliVolts(uint8_t pin, int atten_db = 11, Filter f = Filter{}, Rate rate = Rate{}, Shaper s = Shaper{})
    {
        setEsp32PinAttenuation(pin, atten_db);

        using Reader = ArduinoAnalogReadMilliVolts<FullScaleMv>;
        typename LinearPot<Reader, Filter, Rate, Shaper>::Config cfg;
        cfg.reader = Reader(pin);
        cfg.filter = f;
        cfg.rate = rate;
        cfg.shape = s;
        return LinearPot<Reader, Filter, Rate, Shaper>(cfg);
    }

    /**
     * @brief ESP32 helper: create a millivolt LinearPot with centered calibration.
     *
     * @details
     * Calibration values are millivolts when using this reader. Measure real
     * min/center/max values from your hardware instead of assuming ideal rails.
     *
     * @tparam FullScaleMv Full-scale in millivolts used for normalization.
     * @tparam Filter Filter type.
     * @tparam Rate Rate limiter type.
     * @tparam Shaper Shaper type.
     *
     * @param pin ADC pin.
     * @param c Min/center/max calibration in millivolts.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11).
     * @param f Filter instance.
     * @param rate Rate limiter instance.
     * @param s Shaper instance.
     * @return Configured ESP32 millivolt LinearPot.
     */
    template <int FullScaleMv = 3300, typename Filter = EMAFilter, typename Rate = NoRateLimit, typename Shaper = ShapeIdentity>
    inline LinearPot<ArduinoAnalogReadMilliVolts<FullScaleMv>, Filter, Rate, Shaper>
    makePotMilliVolts(uint8_t pin, PotCalib c, int atten_db = 11, Filter f = Filter{}, Rate rate = Rate{}, Shaper s = Shaper{})
    {
        setEsp32PinAttenuation(pin, atten_db);

        using Reader = ArduinoAnalogReadMilliVolts<FullScaleMv>;
        typename LinearPot<Reader, Filter, Rate, Shaper>::Config cfg;
        cfg.reader = Reader(pin);
        cfg.calib = c;
        cfg.filter = f;
        cfg.rate = rate;
        cfg.shape = s;
        return LinearPot<Reader, Filter, Rate, Shaper>(cfg);
    }
#endif

    // ---- ContinuousPot factories ---- //

    /**
     * @brief Helper: create a ContinuousPot using Arduino analogRead().
     *
     * @tparam Filter Filter type.
     * @tparam Rate Rate limiter type.
     * @tparam Shaper Shaper type.
     *
     * @param pin ADC pin.
     * @param c Min/center/max calibration in raw ADC units.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11). Ignored on non-ESP32.
     * @param f Filter instance.
     * @param r Rate limiter instance.
     * @param s Shaper instance.
     * @return Configured ContinuousPot using ArduinoAnalogRead.
     */
    template <typename Filter = EMAFilter, typename Rate = NoRateLimit, typename Shaper = ShapeIdentity>
    inline ContinuousPot<ArduinoAnalogRead, Filter, Rate, Shaper>
    makeContinuousPot(uint8_t pin,
                      PotCalib c = PotCalib{},
                      int atten_db = 11,
                      Filter f = Filter{}, Rate r = Rate{}, Shaper s = Shaper{})
    {
        setEsp32PinAttenuation(pin, atten_db);

        typename ContinuousPot<ArduinoAnalogRead, Filter, Rate, Shaper>::Config cfg;
        cfg.reader = ArduinoAnalogRead(pin);
        cfg.calib = c;
        cfg.filter = f;
        cfg.rate = r;
        cfg.shape = s;
        return ContinuousPot<ArduinoAnalogRead, Filter, Rate, Shaper>(cfg);
    }

    // ---- Joystick2D factories ---- //

    /**
     * @brief Helper: create a KY-023 style analog joystick (two analog pins).
     *
     * @details
     * ReadX and ReadY are expected to be reader types constructible from
     * `uint8_t pin`, matching ArduinoAnalogRead.
     *
     * @tparam ReadX X-axis reader type.
     * @tparam ReadY Y-axis reader type.
     * @tparam FX X-axis filter type.
     * @tparam FY Y-axis filter type.
     * @tparam RX X-axis rate limiter type.
     * @tparam RY Y-axis rate limiter type.
     * @tparam SX X-axis shaper type.
     * @tparam SY Y-axis shaper type.
     * @tparam ComputeMag Whether to compute magnitude.
     * @tparam ComputeAngle Whether to compute angle.
     *
     * @param pinX ADC pin for X axis.
     * @param pinY ADC pin for Y axis.
     * @param dz Deadzone strategy.
     * @param dz_size Deadzone size in normalized units.
     * @return Configured Joystick2D.
     */
    template <typename ReadX = ArduinoAnalogRead, typename ReadY = ArduinoAnalogRead,
              typename FX = EMAFilter, typename FY = EMAFilter,
              typename RX = NoRateLimit, typename RY = NoRateLimit,
              typename SX = ShapeIdentity, typename SY = ShapeIdentity,
              bool ComputeMag = true, bool ComputeAngle = true>
    inline Joystick2D<ReadX, ReadY, FX, FY, RX, RY, SX, SY, ComputeMag, ComputeAngle>
    makeJoystickKY023(uint8_t pinX, uint8_t pinY,
                      Deadzone dz = Deadzone::RadialScaled,
                      float dz_size = 0.12f)
    {
        typename Joystick2D<ReadX, ReadY, FX, FY, RX, RY, SX, SY, ComputeMag, ComputeAngle>::Config cfg;
        cfg.readX = ReadX(pinX);
        cfg.readY = ReadY(pinY);
        cfg.deadzone = dz;
        cfg.deadzone_size = dz_size;
        return Joystick2D<ReadX, ReadY, FX, FY, RX, RY, SX, SY, ComputeMag, ComputeAngle>(cfg);
    }

    // ---- SteppedPot factories ---- //

    /**
     * @brief Helper: create a SteppedPot using Arduino analogRead().
     *
     * @tparam Steps Number of discrete steps.
     * @tparam Filter Filter type applied before quantization.
     *
     * @param pin ADC pin.
     * @param c Min/center/max calibration in raw ADC units.
     * @param atten_db ESP32 attenuation in dB (0/3/6/11). Ignored on non-ESP32.
     * @param f Filter instance.
     * @param hysteresis Fraction of a step width used as hold band.
     * @return Configured SteppedPot using ArduinoAnalogRead.
     */
    template <size_t Steps, typename Filter = NoFilter>
    inline SteppedPot<Steps, ArduinoAnalogRead, Filter>
    makeSteppedPot(uint8_t pin,
                   PotCalib c = PotCalib{},
                   int atten_db = 11,
                   Filter f = Filter{},
                   float hysteresis = 0.04f)
    {
        setEsp32PinAttenuation(pin, atten_db);

        typename SteppedPot<Steps, ArduinoAnalogRead, Filter>::Config cfg;
        cfg.reader = ArduinoAnalogRead(pin);
        cfg.calib = c;
        cfg.filter = f;
        cfg.hysteresis = hysteresis;
        return SteppedPot<Steps, ArduinoAnalogRead, Filter>(cfg);
    }

} ///< namespace PotIO
