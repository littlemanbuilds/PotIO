/**
 * MIT License
 *
 * @brief Cyclic analog potentiometer with validated acquisition, wrap plausibility, and unwrapped angle tracking.
 *
 * @file PotIO_ContinuousPot.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include "PotIO_Detail.h"
#include "PotIO_Filters.h"
#include "PotIO_RateLimit.h"
#include "PotIO_Shaping.h"
#include "PotIO_Arduino.h"

namespace PotIO
{
    /**
     * @brief Continuous analog control with phase, turn count, and unwrapped angle.
     *
     * @details
     * Wrap tracking is trustworthy only while consecutive valid phase samples
     * remain physically plausible and arrive quickly enough to rule out a hidden
     * half-turn alias. Failed acquisitions do not advance the phase timestamp. A
     * discontinuity invalidates turn tracking until the application explicitly
     * calls resynchronizeTurns().
     */
    template <typename Reader = ArduinoAnalogRead,
              typename Filter = EMAFilter,
              typename Rate = NoRateLimit,
              typename Shaper = ShapeIdentity>
    class ContinuousPot
    {
    public:
        /** @brief Configuration bundle for ContinuousPot. */
        struct Config
        {
            Reader reader{};                                                            ///< ADC reader functor.
            PotCalib calib{};                                                           ///< Min/center/max calibration.
            CalibrationPolicy calibration_policy{CalibrationPolicy::PermissiveDefault}; ///< Calibration failure policy.
            uint16_t min_calibration_span{1u};                                          ///< Minimum raw counts on each side of center.
            Filter filter{};                                                            ///< Filter instance.
            Rate rate{};                                                                ///< Rate limiter instance.
            Shaper shape{};                                                             ///< Response shaper.
            float wrap_hyst{0.05f};                                                     ///< Edge region used for wrap recognition [0,0.45].
            float max_phase_delta{0.45f};                                               ///< Maximum plausible shortest phase delta per sample.
            float max_turns_per_s{8.0f};                                                ///< Maximum plausible velocity; also bounds the unambiguous sample interval.
            float max_dt_s{0.5f};                                                       ///< Largest accepted processing gap in seconds.
            InvalidSamplePolicy invalid_sample_policy{InvalidSamplePolicy::HoldState};  ///< Filter/rate recovery policy.
        };

        /** @brief Public state produced by update(). */
        struct State
        {
            uint16_t raw{0};         ///< Latest retained raw ADC sample.
            float phase01{0.f};      ///< Latest retained normalized phase in [0,1].
            float centered{0.f};     ///< Latest retained final centered output in [-1,1].
            int32_t turns{0};        ///< Signed full turns accumulated since synchronization.
            bool turns_valid{false}; ///< True while wrap tracking remains unambiguous.
            uint32_t t_ms{0};        ///< Timestamp of the latest update attempt.
            SampleStatus status{};   ///< Validity, sample timestamp, sequence, and quality.
        };

        using Frame = State; ///< POD-style latest-state frame.

        ContinuousPot() = default;

        /** @brief Construct with configuration. */
        explicit ContinuousPot(const Config &c) : cfg_(c) { precompute_(); }

        /** @brief Inject a custom millisecond time source. */
        void setTimeSource(TimeFn fn) noexcept { time_fn_ = fn; }

        /** @brief Update using the configured/default time source. */
        void update() noexcept { update(detail::time_now_ms(time_fn_)); }

        /** @brief Update using an external timestamp with wrap-safe internal dt. */
        void update(uint32_t now_ms) noexcept { update(now_ms, dt_.step(now_ms)); }

        /** @brief Acquire and process one cyclic analog sample. */
        void update(uint32_t now_ms, float dt_s) noexcept
        {
            st_.t_ms = now_ms;

            const bool calibration_ok = detail::calibration_valid(cfg_.calib, fs_, cfg_.min_calibration_span);
            const uint16_t quality = calibration_ok ? QualityNone : QualityCalibrationFallback;

            const ReadError config_error = validate_config_();
            if (config_error != ReadError::None)
            {
                fail_(config_error, calibration_ok, quality);
                return;
            }
            if (!calibration_ok && cfg_.calibration_policy == CalibrationPolicy::RequireValid)
            {
                fail_(ReadError::CalibrationInvalid, false, QualityNone);
                return;
            }

            const ReadError timing_error = detail::validate_dt(dt_s, cfg_.max_dt_s);
            if (timing_error != ReadError::None)
            {
                fail_(timing_error, calibration_ok, quality);
                return;
            }

            const RawSample sample = detail::acquire(cfg_.reader, fs_);
            if (!sample.valid)
            {
                fail_(sample.error, calibration_ok, quality);
                return;
            }

            const float phase = detail::map_raw_to_01(sample.raw, cfg_.calib, fs_, calibration_ok);
            if (!wrap_sample_plausible_(phase, now_ms))
            {
                turns_valid_ = false;
                st_.turns_valid = false;
                fail_(ReadError::Discontinuity, calibration_ok, quality);
                return;
            }

            accumulate_wrap_(phase, now_ms);

            const float shaped = cfg_.shape(detail::to_centered(phase));
            if (!detail::finite_float(shaped))
            {
                fail_(ReadError::InvalidConfiguration, calibration_ok, quality);
                return;
            }

            float filtered = shaped;
            float output = shaped;
            if (has_processing_sample_)
            {
                filtered = cfg_.filter(filtered_centered_, shaped);
                if (!detail::finite_float(filtered))
                {
                    fail_(ReadError::InvalidConfiguration, calibration_ok, quality);
                    return;
                }
                output = cfg_.rate(st_.centered, filtered, dt_s);
                if (!detail::finite_float(output))
                {
                    fail_(ReadError::InvalidConfiguration, calibration_ok, quality);
                    return;
                }
            }

            filtered_centered_ = clamp11(filtered);
            has_processing_sample_ = true;

            st_.raw = static_cast<uint16_t>(sample.raw);
            st_.phase01 = clamp01(phase);
            st_.centered = clamp11(output);
            st_.turns = turns_;
            st_.turns_valid = turns_valid_;
            detail::mark_success(st_.status, now_ms, calibration_ok, quality);
        }

        /** @brief Get the latest state. */
        POTIO_NODISCARD const State &state() const noexcept { return st_; }

        /** @brief Get a copy of the latest state. */
        POTIO_NODISCARD Frame frame() const noexcept { return st_; }

        /** @brief True when the latest update produced fresh valid data. */
        POTIO_NODISCARD bool valid() const noexcept { return st_.status.valid; }

        /** @brief True when the accumulated turn count is unambiguous. */
        POTIO_NODISCARD bool turnsValid() const noexcept { return st_.turns_valid; }

        /** @brief Total retained turns since synchronization. */
        POTIO_NODISCARD float turns_raw() const noexcept { return static_cast<float>(turns_); }

        /** @brief Unwrapped angle in degrees. */
        POTIO_NODISCARD float degrees_raw() const noexcept
        {
            return (static_cast<float>(turns_) + st_.phase01) * 360.0f;
        }

        /** @brief Current normalized phase. */
        POTIO_NODISCARD float phase01() const noexcept { return st_.phase01; }

        /** @brief Current phase mapped to [-π,+π]. */
        POTIO_NODISCARD float phaseRad() const noexcept
        {
            return (st_.phase01 * 2.0f - 1.0f) * kPi;
        }

        /** @brief Current phase mapped to [-180,+180] degrees. */
        POTIO_NODISCARD float phaseDeg() const noexcept { return rad2deg(phaseRad()); }

        /** @brief Unwrapped angle in radians. */
        POTIO_NODISCARD float angleUnwrappedRad() const noexcept
        {
            return (static_cast<float>(turns_) + st_.phase01) * (2.0f * kPi);
        }

        /** @brief Unwrapped angle in degrees. */
        POTIO_NODISCARD float angleUnwrappedDeg() const noexcept { return degrees_raw(); }

        /**
         * @brief Replace configuration.
         * @param c New configuration bundle.
         * @param reset_state Clear public state, turn count, and processing history.
         */
        void setConfig(const Config &c, bool reset_state = false) noexcept
        {
            const bool calibration_changed = !detail::calib_equal(cfg_.calib, c.calib) ||
                                             cfg_.calibration_policy != c.calibration_policy ||
                                             cfg_.min_calibration_span != c.min_calibration_span;
            cfg_ = c;
            precompute_();
            if (reset_state)
                resetState();
            else if (calibration_changed)
            {
                reset_processing_();
                resynchronizeTurns(turns_);
            }
        }

        /** @brief Reset accumulated turns to zero while retaining the current wrap phase. */
        void resetTurns() noexcept
        {
            turns_ = 0;
            st_.turns = 0;
        }

        /**
         * @brief Explicitly re-arm wrap tracking after a discontinuity.
         * @param known_turns Turn count the application wants to use as the new reference.
         *
         * @details
         * The next valid sample seeds phase tracking and cannot create a wrap.
         */
        void resynchronizeTurns(int32_t known_turns = 0) noexcept
        {
            turns_ = known_turns;
            st_.turns = known_turns;
            turns_valid_ = false;
            st_.turns_valid = false;
            last_phase01_ = 0.f;
            last_phase_ms_ = 0u;
            has_last_phase_ = false;
        }

        /** @brief Clear public state, turn count, timing, and all processing history. */
        void resetState() noexcept
        {
            st_ = State{};
            dt_ = detail::DtState{};
            turns_ = 0;
            turns_valid_ = false;
            last_phase01_ = 0.f;
            last_phase_ms_ = 0u;
            has_last_phase_ = false;
            reset_processing_();
        }

        /** @brief Get effective reader full-scale. */
        POTIO_NODISCARD int fullScale() const noexcept { return fs_; }

    private:
        void precompute_() noexcept
        {
            const int declared = detail::FullScale<Reader>::value;
            full_scale_valid_ = declared > 0;
            fs_ = full_scale_valid_ ? declared : 1;
        }

        ReadError validate_config_() const noexcept
        {
            if (!full_scale_valid_ || cfg_.min_calibration_span == 0u)
                return ReadError::InvalidConfiguration;
            if (!detail::valid_calibration_policy(cfg_.calibration_policy) ||
                !detail::valid_invalid_sample_policy(cfg_.invalid_sample_policy))
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.wrap_hyst) || cfg_.wrap_hyst < 0.f || cfg_.wrap_hyst > 0.45f)
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.max_phase_delta) || cfg_.max_phase_delta <= 0.f || cfg_.max_phase_delta > 0.5f)
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.max_turns_per_s) || cfg_.max_turns_per_s <= 0.f)
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.max_dt_s) || cfg_.max_dt_s <= 0.f)
                return ReadError::InvalidConfiguration;
            if (!detail::policy_valid(cfg_.filter) || !detail::policy_valid(cfg_.rate) || !detail::policy_valid(cfg_.shape))
                return ReadError::InvalidConfiguration;
            return ReadError::None;
        }

        bool wrap_sample_plausible_(float phase, uint32_t now_ms) const noexcept
        {
            if (!has_last_phase_)
                return true;
            if (!turns_valid_)
                return false;

            float delta = phase - last_phase01_;
            if (delta > 0.5f)
                delta -= 1.0f;
            else if (delta < -0.5f)
                delta += 1.0f;

            const float abs_delta = fabs(delta);
            if (abs_delta > cfg_.max_phase_delta)
                return false;

            // Wrap tracking is only unique while the configured maximum speed
            // cannot cover half a turn between two valid phase samples. Use the
            // last successful phase timestamp so failed acquisitions cannot
            // accidentally shorten this interval.
            const uint32_t elapsed_ms = now_ms - last_phase_ms_;
            const float elapsed_s = 0.001f * static_cast<float>(elapsed_ms);
            if (elapsed_s > kEps)
            {
                if ((cfg_.max_turns_per_s * elapsed_s) >= 0.5f)
                    return false;
                if ((abs_delta / elapsed_s) > cfg_.max_turns_per_s)
                    return false;
            }
            return true;
        }

        void accumulate_wrap_(float phase, uint32_t now_ms) noexcept
        {
            if (!has_last_phase_)
            {
                last_phase01_ = phase;
                last_phase_ms_ = now_ms;
                has_last_phase_ = true;
                turns_valid_ = true;
                return;
            }

            const float h = cfg_.wrap_hyst;
            const bool was_high = last_phase01_ > (1.f - h);
            const bool was_low = last_phase01_ < h;
            const bool is_high = phase > (1.f - h);
            const bool is_low = phase < h;

            if (was_high && is_low)
                ++turns_;
            else if (was_low && is_high)
                --turns_;

            last_phase01_ = phase;
            last_phase_ms_ = now_ms;
        }

        void fail_(ReadError error, bool calibration_ok, uint16_t quality) noexcept
        {
            detail::mark_failure(st_.status, error, calibration_ok, quality);
            if (cfg_.invalid_sample_policy == InvalidSamplePolicy::ResetProcessing)
                reset_processing_();
        }

        void reset_processing_() noexcept
        {
            filtered_centered_ = 0.f;
            has_processing_sample_ = false;
        }

        Config cfg_{};
        State st_{};
        int fs_{POTIO_DEFAULT_FULLSCALE};
        bool full_scale_valid_{true};
        TimeFn time_fn_{nullptr};
        detail::DtState dt_{};
        float filtered_centered_{0.f};
        float last_phase01_{0.f};
        uint32_t last_phase_ms_{0u};
        int32_t turns_{0};
        bool has_processing_sample_{false};
        bool has_last_phase_{false};
        bool turns_valid_{false};
    };

} ///< namespace PotIO
