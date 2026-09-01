/**
 * MIT License
 *
 * @brief Internal acquisition, validation, timing, and mapping helpers for PotIO.
 *
 * @file PotIO_Detail.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include "PotIO_Types.h"

namespace PotIO
{
    namespace detail
    {
        template <typename T, typename = void>
        struct FullScale
        {
            static constexpr int value = POTIO_DEFAULT_FULLSCALE;
        };

        template <typename T>
        struct FullScale<T, decltype((void)T::kFullScale, void())>
        {
            static constexpr int value = T::kFullScale;
        };

        inline bool finite_float(float value) noexcept
        {
            return isfinite(value) != 0;
        }

        inline RawSample normalize_sample(int raw) noexcept
        {
            return (raw < 0) ? RawSample::failure(ReadError::ReaderFailure)
                             : RawSample::success(raw);
        }

        inline RawSample normalize_sample(const RawSample &sample) noexcept
        {
            if (!sample.valid)
                return RawSample::failure(sample.error);
            return sample;
        }

        template <typename Reader>
        inline RawSample acquire(Reader &reader, int fullscale) noexcept
        {
            RawSample sample = normalize_sample(reader());
            if (!sample.valid)
                return sample;
            if (sample.raw < 0)
                return RawSample::failure(ReadError::ReaderFailure);
            if (sample.raw > fullscale)
                return RawSample::failure(ReadError::OutOfRange);
            return sample;
        }

        inline uint32_t time_now_ms(TimeFn fn) noexcept
        {
            if (fn)
                return fn();
#if defined(ARDUINO)
            return static_cast<uint32_t>(::millis());
#else
            return 0u;
#endif
        }

        struct DtState
        {
            uint32_t last_ms{0};
            bool has_last{false};

            inline float step(uint32_t now_ms) noexcept
            {
                if (!has_last)
                {
                    has_last = true;
                    last_ms = now_ms;
                    return 0.0f;
                }
                const uint32_t dt_ms = now_ms - last_ms;
                last_ms = now_ms;
                return 0.001f * static_cast<float>(dt_ms);
            }
        };

        inline ReadError validate_dt(float dt_s, float max_dt_s) noexcept
        {
            if (!finite_float(dt_s) || dt_s < 0.0f)
                return ReadError::InvalidTiming;
            if (!finite_float(max_dt_s) || max_dt_s <= 0.0f)
                return ReadError::InvalidConfiguration;
            if (dt_s > max_dt_s)
                return ReadError::TimingGap;
            return ReadError::None;
        }

        inline bool calibration_valid(const PotCalib &calib, int fullscale, uint16_t min_span) noexcept
        {
            return calib.valid_centered_for(fullscale, min_span);
        }

        inline bool valid_calibration_policy(CalibrationPolicy policy) noexcept
        {
            return policy == CalibrationPolicy::PermissiveDefault ||
                   policy == CalibrationPolicy::RequireValid;
        }

        inline bool valid_invalid_sample_policy(InvalidSamplePolicy policy) noexcept
        {
            return policy == InvalidSamplePolicy::HoldState ||
                   policy == InvalidSamplePolicy::ResetProcessing;
        }

        inline bool valid_deadzone(Deadzone deadzone) noexcept
        {
            return deadzone == Deadzone::None || deadzone == Deadzone::Axial ||
                   deadzone == Deadzone::AxialScaled || deadzone == Deadzone::Radial ||
                   deadzone == Deadzone::RadialScaled;
        }

        inline bool valid_geometry(JoystickGeometry geometry) noexcept
        {
            return geometry == JoystickGeometry::Square ||
                   geometry == JoystickGeometry::MagnitudeClamp ||
                   geometry == JoystickGeometry::SquareToCircle;
        }

        inline float map_raw_to_01(int raw,
                                   const PotCalib &calib,
                                   int fullscale,
                                   bool use_calibration) noexcept
        {
            const int fs = (fullscale > 0) ? fullscale : 1;
            const int r = (raw < 0) ? 0 : ((raw > fs) ? fs : raw);

            if (use_calibration)
            {
                const int cal_min = static_cast<int>(calib.min);
                const int cal_center = static_cast<int>(calib.center);
                const int cal_max = static_cast<int>(calib.max);

                if (r <= cal_center)
                {
                    const float span = static_cast<float>(cal_center - cal_min);
                    return clamp01(0.5f * static_cast<float>(r - cal_min) / span);
                }
                const float span = static_cast<float>(cal_max - cal_center);
                return clamp01(0.5f + 0.5f * static_cast<float>(r - cal_center) / span);
            }

            return clamp01(static_cast<float>(r) / static_cast<float>(fs));
        }

        inline float to_centered(float v01) noexcept
        {
            return (clamp01(v01) - 0.5f) * 2.0f;
        }

        template <typename Policy>
        inline auto policy_valid_impl(const Policy &policy, int) noexcept -> decltype(policy.valid(), bool())
        {
            return policy.valid();
        }

        template <typename Policy>
        inline bool policy_valid_impl(const Policy &, long) noexcept
        {
            return true;
        }

        template <typename Policy>
        inline bool policy_valid(const Policy &policy) noexcept
        {
            return policy_valid_impl(policy, 0);
        }

        inline void mark_failure(SampleStatus &status,
                                 ReadError error,
                                 bool calibration_is_valid,
                                 uint16_t quality = QualityNone) noexcept
        {
            status.valid = false;
            status.calibration_valid = calibration_is_valid;
            status.error = (error == ReadError::None) ? ReadError::ReaderFailure : error;
            status.quality = quality;
        }

        inline void mark_success(SampleStatus &status,
                                 uint32_t now_ms,
                                 bool calibration_is_valid,
                                 uint16_t quality = QualityNone) noexcept
        {
            status.valid = true;
            status.has_value = true;
            status.calibration_valid = calibration_is_valid;
            status.error = ReadError::None;
            status.quality = quality;
            status.sample_ms = now_ms;
            ++status.sequence;
        }

        inline bool calib_equal(const PotCalib &a, const PotCalib &b) noexcept
        {
            return a.min == b.min && a.center == b.center && a.max == b.max;
        }

    } ///< namespace detail
} ///< namespace PotIO
