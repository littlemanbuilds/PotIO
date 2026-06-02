/**
 * MIT License
 *
 * @brief Jitter tools to derive sensible deadzones from ADC noise.
 *
 * @file PotIO_JitterTools.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include <PotIO_Compatibility.h>

namespace PotIO
{
    /**
     * @brief Track min/max over observations to compute peak-to-peak jitter.
     *
     * @note JitterStats accumulates observed min/max until reset() is called or
     * a fresh instance is created.
     */
    struct JitterStats
    {
        uint16_t min{65535}; ///< Minimum observed value.
        uint16_t max{0};     ///< Maximum observed value.

        /**
         * @brief Observe a new sample and update jitter range.
         * @param v New raw sample.
         */
        void observe(uint16_t v) noexcept
        {
            if (v < min)
                min = v;
            if (v > max)
                max = v;
        }

        /**
         * @brief Clear the observed range.
         */
        void reset() noexcept
        {
            min = 65535;
            max = 0;
        }

        /**
         * @brief Peak-to-peak jitter.
         * @return uint16_t Range (max - min), or 0 if underflow.
         */
        POTIO_NODISCARD uint16_t peak_to_peak() const noexcept
        {
            return (max >= min) ? static_cast<uint16_t>(max - min) : 0;
        }
    };

    /**
     * @brief Suggest a radial deadzone (fraction of full-scale) from jitter.
     *
     * @param jitter_pp Peak-to-peak jitter in raw units.
     * @param fullscale ADC full-scale (e.g., 4095 or 1023).
     * @param safety_margin Extra margin in raw units added to jitter.
     * @return float Deadzone in [0, 0.4].
     */
    inline float suggest_deadzone_from_jitter(uint16_t jitter_pp,
                                              uint16_t fullscale = static_cast<uint16_t>(POTIO_DEFAULT_FULLSCALE),
                                              uint16_t safety_margin = 5) noexcept
    {
        const float j = static_cast<float>(jitter_pp + safety_margin);
        const float dz = j / static_cast<float>((fullscale > 0) ? fullscale : 1u);
        return (dz < 0.f) ? 0.f : (dz > 0.4f ? 0.4f : dz);
    }

} ///< namespace PotIO
