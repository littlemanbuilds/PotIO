/**
 * MIT License
 *
 * @brief Bounded jitter tools for deriving practical analog deadzones.
 *
 * @file PotIO_JitterTools.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "PotIO_Compatibility.h"

namespace PotIO
{
    /**
     * @brief Track recent raw ADC jitter over a fixed rolling window.
     *
     * @tparam Window Number of recent samples retained. Must be at least 2.
     *
     * @details
     * Unlike an all-time min/max, one startup spike eventually leaves the
     * window and no longer dominates the deadzone estimate. Storage is fixed
     * at compile time and no dynamic allocation is used.
     */
    template <size_t Window = 32u>
    class RollingJitterStats
    {
        static_assert(Window >= 2u, "RollingJitterStats window must be >= 2.");

    public:
        /** @brief Observe one valid raw sample. */
        void observe(uint16_t value) noexcept
        {
            samples_[head_] = value;
            head_ = (head_ + 1u) % Window;
            if (count_ < Window)
                ++count_;
        }

        /** @brief Clear the rolling window. */
        void reset() noexcept
        {
            head_ = 0u;
            count_ = 0u;
        }

        /** @brief Number of observations currently represented. */
        POTIO_NODISCARD size_t count() const noexcept { return count_; }

        /** @brief True once the window contains Window observations. */
        POTIO_NODISCARD bool full() const noexcept { return count_ == Window; }

        /** @brief Minimum recent value, or zero before the first observation. */
        POTIO_NODISCARD uint16_t min_value() const noexcept
        {
            if (count_ == 0u)
                return 0u;
            uint16_t value = samples_[0];
            for (size_t i = 1u; i < count_; ++i)
            {
                if (samples_[i] < value)
                    value = samples_[i];
            }
            return value;
        }

        /** @brief Maximum recent value, or zero before the first observation. */
        POTIO_NODISCARD uint16_t max_value() const noexcept
        {
            if (count_ == 0u)
                return 0u;
            uint16_t value = samples_[0];
            for (size_t i = 1u; i < count_; ++i)
            {
                if (samples_[i] > value)
                    value = samples_[i];
            }
            return value;
        }

        /** @brief Recent peak-to-peak jitter. */
        POTIO_NODISCARD uint16_t peak_to_peak() const noexcept
        {
            if (count_ < 2u)
                return 0u;
            const uint16_t lo = min_value();
            const uint16_t hi = max_value();
            return static_cast<uint16_t>(hi - lo);
        }

    private:
        uint16_t samples_[Window]{}; ///< Fixed rolling sample storage.
        size_t head_{0u};            ///< Index overwritten by the next observation.
        size_t count_{0u};           ///< Number of populated entries.
    };

    /**
     * @brief Backward-compatible default jitter tracker using 32 recent samples.
     *
     * @details
     * v1.0.0 accumulated lifetime min/max. v1.1.0 intentionally makes the
     * default diagnostic representative of current noise instead.
     */
    using JitterStats = RollingJitterStats<32u>;

    /**
     * @brief Suggest a deadzone fraction from recent peak-to-peak jitter.
     * @param jitter_pp Peak-to-peak jitter in raw units.
     * @param fullscale ADC full-scale.
     * @param safety_margin Extra raw-count margin.
     * @return Deadzone fraction clamped to [0, 0.4].
     */
    inline float suggest_deadzone_from_jitter(uint16_t jitter_pp,
                                              uint16_t fullscale = static_cast<uint16_t>(POTIO_DEFAULT_FULLSCALE),
                                              uint16_t safety_margin = 5u) noexcept
    {
        const uint32_t raw_margin = static_cast<uint32_t>(jitter_pp) + static_cast<uint32_t>(safety_margin);
        const float denom = static_cast<float>((fullscale > 0u) ? fullscale : 1u);
        const float dz = static_cast<float>(raw_margin) / denom;
        return (dz > 0.4f) ? 0.4f : dz;
    }

} ///< namespace PotIO
