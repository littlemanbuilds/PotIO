/**
 * MIT License
 *
 * @brief Allocation-free rate-limit primitives for PotIO.
 *
 * @file PotIO_RateLimit.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <math.h>

namespace PotIO
{
    /** @brief No rate limiting; passes target through. */
    struct NoRateLimit
    {
        /**
 * @brief Validate configuration.
 *
 * @return Always true.
 */
        bool valid() const noexcept { return true; }

        /** @brief Apply rate limiting. */
        float operator()(float prev, float target, float dt) const noexcept
        {
            (void)dt;
            (void)prev;
            return target;
        }
    };

    /** @brief Symmetric slew-rate limiter in normalized units per second. */
    struct SlewRate
    {
        float units_per_s{2.0f}; ///< Maximum non-negative change per second.

        /** @brief Validate the configured slew rate. */
        bool valid() const noexcept
        {
            return isfinite(units_per_s) != 0 && units_per_s >= 0.0f;
        }

        /** @brief Apply slew-rate limiting towards target. */
        float operator()(float prev, float target, float dt) const noexcept
        {
            if (!valid() || isfinite(dt) == 0 || dt < 0.0f)
                return prev;

            const float max_step = units_per_s * dt;
            const float d = target - prev;
            if (d > max_step)
                return prev + max_step;
            if (d < -max_step)
                return prev - max_step;
            return target;
        }
    };

} ///< namespace PotIO
