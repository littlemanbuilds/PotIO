/**
 * MIT License
 *
 * @brief Rate limit primitives for shaping signals in time.
 *
 * @file PotIO_RateLimit.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

namespace PotIO
{
    /**
     * @brief No rate limiting; passes target through.
     */
    struct NoRateLimit
    {
        /**
         * @brief Apply rate limiting.
         * @param prev Previous output value.
         * @param target Target value to approach.
         * @param dt Elapsed time in seconds.
         * @return float New output value.
         */
        float operator()(float prev, float target, float dt) const noexcept
        {
            (void)dt;
            (void)prev;
            return target;
        }
    };

    /**
     * @brief Slew rate limiter with symmetric bound (units per second).
     */
    struct SlewRate
    {
        float units_per_s{2.0f}; ///< Maximum change per second.

        /**
         * @brief Apply slew rate limiting towards target.
         * @param prev Previous output value.
         * @param target Target value to approach.
         * @param dt Elapsed time in seconds.
         * @return float New output value.
         */
        float operator()(float prev, float target, float dt) const noexcept
        {
            const float safe_dt = (dt < 0.0f) ? 0.0f : dt;
            const float max_step = units_per_s * safe_dt;
            const float d = target - prev;
            if (d > max_step)
                return prev + max_step;
            if (d < -max_step)
                return prev - max_step;
            return target;
        }
    };

} ///< namespace PotIO
