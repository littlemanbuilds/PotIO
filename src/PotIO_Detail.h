/**
 * MIT License
 *
 * @brief Internal helpers shared by PotIO devices (not part of the public API).
 *
 * @file PotIO_Detail.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include <PotIO_Types.h>

namespace PotIO
{
namespace detail
{
    // ---- Reader full-scale detection ---- //

    /**
     * @brief Default reader full-scale trait.
     *
     * @tparam T Reader type.
     * @note Readers can override this by exposing `static constexpr int kFullScale`.
     */
    template <typename T, typename = void>
    struct FullScale
    {
        static constexpr int value = POTIO_DEFAULT_FULLSCALE;
    };

    /**
     * @brief Reader full-scale specialization for readers with `T::kFullScale`.
     * @tparam T Reader type that declares `kFullScale`.
     */
    template <typename T>
    struct FullScale<T, decltype((void)T::kFullScale, void())>
    {
        static constexpr int value = T::kFullScale;
    };

    // ---- Time helpers ---- //

    /**
     * @brief Get "now" in milliseconds from an injected time source (or Arduino ::millis()).
     * @param fn Optional millisecond time source.
     * @return Current time in milliseconds, or 0 in non-Arduino builds without a time source.
     */
    inline uint32_t time_now_ms(TimeFn fn) noexcept
    {
        if (fn)
            return fn();
#if defined(ARDUINO)
        return static_cast<uint32_t>(::millis());
#else
        // Non-Arduino builds should inject a TimeFn for meaningful timestamps.
        return 0u;
#endif
    }

    /**
     * @brief Wrap-safe dt computation with a first-call guard.
     */
    struct DtState
    {
        uint32_t last_ms{0};
        bool has_last{false};

        /**
         * @brief Return dt (seconds) and update internal last timestamp.
         * @param now_ms Current timestamp in milliseconds.
         * @return Elapsed seconds since the previous step, or 0 on the first call.
         */
        inline float step(uint32_t now_ms) noexcept
        {
            if (!has_last)
            {
                has_last = true;
                last_ms = now_ms;
                return 0.0f;
            }
            const uint32_t dt_ms = now_ms - last_ms; // wrap-safe uint32_t math
            last_ms = now_ms;
            return 0.001f * static_cast<float>(dt_ms);
        }
    };

    // ---- Calibration mapping ---- //

    /**
     * @brief Clamp raw reading to [0, fullscale].
     * @param raw Raw reader sample.
     * @param fullscale Maximum raw value expected from the reader.
     * @return Raw sample constrained to the valid ADC range.
     */
    inline int clamp_raw(int raw, int fullscale) noexcept
    {
        if (raw < 0)
            return 0;
        if (raw > fullscale)
            return fullscale;
        return raw;
    }

    /**
     * @brief Map a raw sample to [0..1] using centered calibration if valid, otherwise using full-scale.
     * @param raw Raw reader sample.
     * @param calib Optional min/center/max calibration.
     * @param fullscale Maximum raw value expected from the reader.
     * @return Normalized value in [0,1].
     * @note Invalid or incomplete calibration falls back to simple full-scale mapping.
     */
    inline float map_raw_to_01(int raw, const PotCalib &calib, int fullscale) noexcept
    {
        const int fs = (fullscale > 0) ? fullscale : 1;
        const int r = clamp_raw(raw, fs);

        if (calib.valid_centered())
        {
            const int cal_min = static_cast<int>(calib.min);
            const int cal_center = static_cast<int>(calib.center);
            const int cal_max = static_cast<int>(calib.max);

            if (r <= cal_center)
            {
                const float span = static_cast<float>(cal_center - cal_min);
                const float inv = (span > kEps) ? (1.0f / span) : 0.0f;
                return clamp01(0.5f * static_cast<float>(r - cal_min) * inv);
            }
            const float span = static_cast<float>(cal_max - cal_center);
            const float inv = (span > kEps) ? (1.0f / span) : 0.0f;
            return clamp01(0.5f + 0.5f * static_cast<float>(r - cal_center) * inv);
        }

        // Simple normalization when not calibrated (or when calibration is incomplete).
        return clamp01(static_cast<float>(r) / static_cast<float>(fs));
    }

    /**
     * @brief Map a normalized [0..1] to centered [-1..1].
     * @param v01 Normalized value.
     * @return Centered value in [-1,1].
     */
    inline float to_centered(float v01) noexcept { return (clamp01(v01) - 0.5f) * 2.0f; }

} ///< namespace detail
} ///< namespace PotIO
