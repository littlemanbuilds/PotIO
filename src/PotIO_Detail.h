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

        /**
         * @brief Test whether a processing value is finite.
         *
         * @param value Value to classify.
         * @return False for NaN or either infinity.
         */
        inline bool finite_float(float value) noexcept
        {
            return isfinite(value) != 0;
        }

        /**
         * @brief Adapt an integer reader result to explicit acquisition status.
         *
         * @param raw Reader result; negative values report failure.
         * @return Successful non-negative sample or ReaderFailure.
         */
        inline RawSample normalize_sample(int raw) noexcept
        {
            return (raw < 0) ? RawSample::failure(ReadError::ReaderFailure)
                             : RawSample::success(raw);
        }

        /**
         * @brief Normalize an explicit reader failure reason.
         *
         * @param sample Reader result to normalize.
         * @return Valid sample unchanged, or a failure whose reason cannot be None.
         */
        inline RawSample normalize_sample(const RawSample &sample) noexcept
        {
            if (!sample.valid)
                return RawSample::failure(sample.error);
            return sample;
        }

        /**
         * @brief Acquire once and reject failures before any mapping can clamp them.
         *
         * @tparam Reader Callable returning int or RawSample.
         * @param reader Reader instance, invoked once.
         * @param fullscale Maximum accepted raw value.
         * @return Sample in [0,fullscale], or an explicit reader/range failure.
         */
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

        /**
         * @brief Read the selected millisecond clock.
         *
         * @param fn Injected clock, or nullptr to use the platform default.
         * @return Injected time, Arduino millis(), or zero on a non-Arduino host.
         */
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

            /**
             * @brief Advance the attempt clock and calculate wrap-safe elapsed time.
             *
             * @param now_ms Current millisecond tick, sampled less than one full rollover apart.
             * @return Elapsed seconds since the preceding attempt, or zero on the first call.
             */
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

        /**
         * @brief Validate processing time and the configured maximum gap.
         *
         * @param dt_s Elapsed seconds; zero is allowed.
         * @param max_dt_s Finite positive maximum processing gap in seconds.
         * @return InvalidTiming, InvalidConfiguration, TimingGap, or None, in that priority.
         */
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

        /**
         * @brief Validate centered calibration against the acquisition range.
         *
         * @param calib Calibration to check.
         * @param fullscale Reader maximum raw value.
         * @param min_span Minimum raw span on each side of center.
         * @return True for ordered, in-range calibration with sufficient spans.
         */
        inline bool calibration_valid(const PotCalib &calib, int fullscale, uint16_t min_span) noexcept
        {
            return calib.valid_centered_for(fullscale, min_span);
        }

        /**
         * @brief Reject unrecognized calibration policy values.
         *
         * @param policy Policy to check.
         * @return True for either supported calibration policy.
         */
        inline bool valid_calibration_policy(CalibrationPolicy policy) noexcept
        {
            return policy == CalibrationPolicy::PermissiveDefault ||
                   policy == CalibrationPolicy::RequireValid;
        }

        /**
         * @brief Reject unrecognized recovery policy values.
         *
         * @param policy Policy to check.
         * @return True for HoldState or ResetProcessing.
         */
        inline bool valid_invalid_sample_policy(InvalidSamplePolicy policy) noexcept
        {
            return policy == InvalidSamplePolicy::HoldState ||
                   policy == InvalidSamplePolicy::ResetProcessing;
        }

        /**
         * @brief Reject unrecognized joystick deadzone modes.
         *
         * @param deadzone Mode to check.
         * @return True for a declared Deadzone enumerator.
         */
        inline bool valid_deadzone(Deadzone deadzone) noexcept
        {
            return deadzone == Deadzone::None || deadzone == Deadzone::Axial ||
                   deadzone == Deadzone::AxialScaled || deadzone == Deadzone::Radial ||
                   deadzone == Deadzone::RadialScaled;
        }

        /**
         * @brief Reject unrecognized joystick geometry modes.
         *
         * @param geometry Mode to check.
         * @return True for a declared JoystickGeometry enumerator.
         */
        inline bool valid_geometry(JoystickGeometry geometry) noexcept
        {
            return geometry == JoystickGeometry::Square ||
                   geometry == JoystickGeometry::MagnitudeClamp ||
                   geometry == JoystickGeometry::SquareToCircle;
        }

        /**
         * @brief Map a raw sample around the physical calibration center or reader range.
         *
         * @param raw Raw sample, clamped to the effective reader range.
         * @param calib Centered calibration, required to be valid when use_calibration is true.
         * @param fullscale Reader maximum; non-positive values use one defensively.
         * @param use_calibration Select piecewise mapping with center at 0.5; otherwise use full-scale mapping.
         * @return Normalized value clamped to [0,1].
         */
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

        /**
         * @brief Convert a normalized unipolar value to a centered coordinate.
         *
         * @param v01 Normalized value; must be finite.
         * @return Value in [-1,1], with 0.5 mapped to zero.
         */
        inline float to_centered(float v01) noexcept
        {
            return (clamp01(v01) - 0.5f) * 2.0f;
        }

        /**
         * @brief Use a policy validation hook when one is available.
         *
         * @tparam Policy Policy type providing valid().
         * @param policy Policy instance to validate.
         * @return The result of policy.valid(), converted to bool.
         */
        template <typename Policy>
        inline auto policy_valid_impl(const Policy &policy, int) noexcept -> decltype(policy.valid(), bool())
        {
            return policy.valid();
        }

        /**
         * @brief Accept legacy policy types that provide no validation hook.
         *
         * @tparam Policy Policy type without a usable valid() member.
         * @return Always true; output finiteness remains the device responsibility.
         */
        template <typename Policy>
        inline bool policy_valid_impl(const Policy &, long) noexcept
        {
            return true;
        }

        /**
         * @brief Validate a policy while retaining support for legacy functors.
         *
         * @tparam Policy Filter, rate, or shaping policy type.
         * @param policy Policy instance to validate.
         * @return valid() when supplied, otherwise true.
         */
        template <typename Policy>
        inline bool policy_valid(const Policy &policy) noexcept
        {
            return policy_valid_impl(policy, 0);
        }

        /**
         * @brief Record failure while retaining the last successful acquisition identity.
         *
         * @param status Status to update; has_value, sample_ms and sequence are retained.
         * @param error Failure reason; None is converted to ReaderFailure.
         * @param calibration_is_valid Whether configured calibration passed validation.
         * @param quality Quality flags for this attempt.
         */
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

        /**
         * @brief Publish successful processing and advance the sample sequence.
         *
         * @param status Status to update.
         * @param now_ms Timestamp of this successful sample.
         * @param calibration_is_valid Whether configured calibration passed validation.
         * @param quality Quality flags for this sample.
         */
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

        /**
         * @brief Compare stored calibration coordinates exactly.
         *
         * @param a First calibration.
         * @param b Second calibration.
         * @return True when all three raw coordinates match.
         */
        inline bool calib_equal(const PotCalib &a, const PotCalib &b) noexcept
        {
            return a.min == b.min && a.center == b.center && a.max == b.max;
        }

    } ///< namespace detail
} ///< namespace PotIO
