/**
 * MIT License
 *
 * @brief Simple filter primitives for PotIO.
 *
 * @file PotIO_Filters.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

namespace PotIO
{
    /**
     * @brief No-op filter; returns the sample unchanged.
     */
    struct NoFilter
    {
        /**
         * @brief Apply the filter to a sample.
         * @param prev Previous filtered value.
         * @param sample New input sample.
         * @return float Filtered output (equal to `sample`).
         */
        float operator()(float prev, float sample) const noexcept
        {
            (void)prev;
            return sample;
        }
    };

    /**
     * @brief Exponential moving average filter.
     */
    struct EMAFilter
    {
        float alpha{0.1f}; ///< Smoothing factor in (0,1]. Lower is smoother.

        /**
         * @brief Apply EMA to the input.
         * @param prev Previous filtered value.
         * @param sample New input sample.
         * @return float Filtered output.
         */
        float operator()(float prev, float sample) const noexcept
        {
            float a = alpha;
            if (a < 0.0f)
                a = 0.0f;
            if (a > 1.0f)
                a = 1.0f;
            return prev + a * (sample - prev);
        }
    };

} ///< namespace PotIO
