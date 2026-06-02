/**
 * MIT License
 *
 * @brief Core types, constants, and small utilities for PotIO.
 *
 * @file PotIO_Types.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include <PotIO_Compatibility.h>

namespace PotIO
{
    /**
     * @brief Time source function type (milliseconds).
     *
     * @details
     * The signature is `uint32_t()` for easy cross-platform integration.
     * When running under Arduino, `nullptr` time sources default to ::millis().
     */
    using TimeFn = uint32_t (*)();

    /// @brief Small epsilon used for float comparisons.
    static constexpr float kEps = 1e-7f;

    // ---- Angle helpers ---- //

    static constexpr float kPi = 3.14159265358979323846f;    ///< π constant.
    static constexpr float kRad2Deg = 57.29577951308232f;    ///< Radians-to-degrees scale (180/π).
    static constexpr float kDeg2Rad = 0.017453292519943295f; ///< Degrees-to-radians scale (π/180).

    /**
     * @brief Convert radians to degrees.
     * @param r Angle in radians.
     * @return Angle in degrees.
     */
    inline float rad2deg(float r) noexcept { return r * kRad2Deg; }

    /**
     * @brief Convert degrees to radians.
     * @param d Angle in degrees.
     * @return Angle in radians.
     */
    inline float deg2rad(float d) noexcept { return d * kDeg2Rad; }

    // ---- Calibration ---- //

    /**
     * @brief Potentiometer calibration (min/center/max).
     *
     * @details
     * PotIO supports "centered" calibration for sticks/pots where the physical
     * center is meaningful (e.g., joystick axes). When calibration is valid,
     * mapping is piecewise-linear:
     *
     *  - [min .. center] -> [0.0 .. 0.5]
     *  - [center .. max] -> [0.5 .. 1.0]
     */
    struct PotCalib
    {
        uint16_t min{0};                                            ///< Raw minimum value.
        uint16_t center{static_cast<uint16_t>(POTIO_DEFAULT_FULLSCALE / 2)}; ///< Raw center value.
        uint16_t max{static_cast<uint16_t>(POTIO_DEFAULT_FULLSCALE)};        ///< Raw maximum value.

        PotCalib() = default;

        /**
         * @brief Construct calibration from observed raw endpoints.
         * @param min_ Raw reading at the low end of travel.
         * @param center_ Raw reading at the physical center.
         * @param max_ Raw reading at the high end of travel.
         */
        PotCalib(uint16_t min_, uint16_t center_, uint16_t max_) noexcept
            : min{min_}, center{center_}, max{max_} {}

        /**
         * @brief True if the calibration range is usable.
         * @return true when `max > min`.
         */
        POTIO_NODISCARD bool valid() const noexcept { return max > min; }

        /**
         * @brief True if calibration looks centered and usable (min < center < max).
         * @return true when min, center, and max form a valid centered calibration.
         */
        POTIO_NODISCARD bool valid_centered() const noexcept { return (max > center) && (center > min); }
    };

    /// @brief Deadzone strategies for joysticks.
    enum class Deadzone : uint8_t
    {
        None,         ///< No deadzone.
        Axial,        ///< Apply deadzone independently to X/Y axes.
        Radial,       ///< Circular deadzone, preserves angle but not magnitude scaling.
        RadialScaled  ///< Circular deadzone and rescale magnitude to reach full-scale outside the zone.
    };

    /**
     * @brief Clamp a value to [0,1].
     * @param v Input value.
     * @return `v` clamped to the normalized range.
     */
    inline float clamp01(float v) noexcept { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

    /**
     * @brief Clamp a value to [-1,1].
     * @param v Input value.
     * @return `v` clamped to the centered range.
     */
    inline float clamp11(float v) noexcept { return v < -1.f ? -1.f : (v > 1.f ? 1.f : v); }

} ///< namespace PotIO
