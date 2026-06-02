/**
 * MIT License
 *
 * @brief Simple facade for a 2-axis joystick with sensible defaults.
 *
 * @file PotIO_AnalogStick.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include <devices/PotIO_Joystick2D.h>
#include <PotIO_Arduino.h>
#include <PotIO_Filters.h>
#include <PotIO_RateLimit.h>
#include <PotIO_Shaping.h>
#include <PotIO_Factory.h>

namespace PotIO
{
    /**
     * @brief Minimal wrapper around Joystick2D with defaults for Arduino KY-023 style sticks.
     *
     * @details
     * This facade exists for "drop it in" usage in quick sketches.
     * For advanced customization, use Joystick2D directly.
     */
    struct AnalogStick
    {
        using Joy = Joystick2D<
            ArduinoAnalogRead, ArduinoAnalogRead,
            EMAFilter, EMAFilter,
            NoRateLimit, NoRateLimit,
            ShapeIdentity, ShapeIdentity,
            true, true>;

        Joy joy; ///< Concrete joystick instance.

        /**
         * @brief Construct a simple joystick.
         *
         * @param x_pin Analog pin for X axis.
         * @param y_pin Analog pin for Y axis.
         * @param deadzone Deadzone fraction (radial scaled), default 0.12.
         */
        explicit AnalogStick(uint8_t x_pin, uint8_t y_pin, float deadzone = 0.12f)
            : joy(makeJoystickKY023<>(x_pin, y_pin, Deadzone::RadialScaled, deadzone)) {}

        /**
         * @brief Inject a custom time source for the underlying joystick.
         * @param fn Optional function returning current milliseconds.
         */
        void setTimeSource(TimeFn fn) noexcept { joy.setTimeSource(fn); }

        /**
         * @brief Update using the current time source.
         */
        void update() noexcept { joy.update(); }

        /**
         * @brief Update using an external timestamp.
         * @param now_ms Current timestamp in milliseconds.
         */
        void update(uint32_t now_ms) noexcept { joy.update(now_ms); }

        /**
         * @brief Update using explicit timestamp and elapsed time.
         * @param now_ms Current timestamp in milliseconds.
         * @param dt_s Elapsed time in seconds.
         */
        void update(uint32_t now_ms, float dt_s) noexcept { joy.update(now_ms, dt_s); }

        /**
         * @brief Clear output history and timing state.
         */
        void resetState() noexcept { joy.resetState(); }

        /**
         * @brief Get latest X-axis value.
         * @return X in [-1,1].
         */
        POTIO_NODISCARD float x() const noexcept { return joy.state().x; }

        /**
         * @brief Get latest Y-axis value.
         * @return Y in [-1,1].
         */
        POTIO_NODISCARD float y() const noexcept { return joy.state().y; }

        /**
         * @brief Get latest radial magnitude.
         * @return Magnitude in [0,1].
         */
        POTIO_NODISCARD float mag() const noexcept { return joy.state().mag; }

        /**
         * @brief Get latest joystick angle.
         * @return Angle in radians.
         */
        POTIO_NODISCARD float angle() const noexcept { return joy.state().angle; }
    };

} ///< namespace PotIO
