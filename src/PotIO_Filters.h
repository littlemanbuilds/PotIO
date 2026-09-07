/**
 * MIT License
 *
 * @brief Small, allocation-free filter primitives for PotIO.
 *
 * @file PotIO_Filters.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <math.h>

namespace PotIO
{
    /** @brief No-op filter; returns the sample unchanged. */
    struct NoFilter
    {
        /**
         * @brief Validate configuration.
         *
         * @return Always true.
         */
        bool valid() const noexcept { return true; }

        /**
         * @brief Pass through a sample without using history.
         *
         * @param prev Previous output, ignored.
         * @param sample New sample.
         * @return sample unchanged.
         */
        float operator()(float prev, float sample) const noexcept
        {
            (void)prev;
            return sample;
        }
    };

    /** @brief Exponential moving average filter. */
    struct EMAFilter
    {
        float alpha{0.1f}; ///< Smoothing factor in [0,1]. Lower is smoother.

        /**
         * @brief Validate the configured smoothing factor.
         *
         * @return True for finite alpha in [0,1].
         */
        bool valid() const noexcept
        {
            return isfinite(alpha) != 0 && alpha >= 0.0f && alpha <= 1.0f;
        }

        /**
         * @brief Blend a sample with the previous filtered value.
         *
         * @param prev Previous filtered value.
         * @param sample New sample in the same units.
         * @return EMA result, or prev when alpha is invalid.
         */
        float operator()(float prev, float sample) const noexcept
        {
            if (!valid())
                return prev;
            return prev + alpha * (sample - prev);
        }
    };

} ///< namespace PotIO
