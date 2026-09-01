/**
 * MIT License
 *
 * @brief Calibrated linear potentiometer with explicit validity, filtering, shaping, and slew limiting.
 *
 * @file PotIO_LinearPot.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include "PotIO_Detail.h"
#include "PotIO_Filters.h"
#include "PotIO_RateLimit.h"
#include "PotIO_Shaping.h"

namespace PotIO
{
    /**
     * @brief Linear potentiometer device: acquire → calibrate → shape → filter → rate limit.
     *
     * @details
     * Call update() from one context. Invalid acquisition never becomes a valid
     * extreme command: the last good output is retained and State::status.valid
     * becomes false. Cross-task transport remains the responsibility of the host
     * application.
     */
    template <typename Reader,
              typename Filter = EMAFilter,
              typename Rate = NoRateLimit,
              typename Shaper = ShapeIdentity>
    class LinearPot
    {
    public:
        /** @brief Configuration bundle for LinearPot. */
        struct Config
        {
            Reader reader{};                                                            ///< ADC reader functor.
            PotCalib calib{};                                                           ///< Min/center/max calibration.
            CalibrationPolicy calibration_policy{CalibrationPolicy::PermissiveDefault}; ///< Calibration failure policy.
            uint16_t min_calibration_span{1u};                                          ///< Minimum raw counts on each side of center.
            float max_dt_s{0.5f};                                                       ///< Largest accepted processing gap in seconds.
            InvalidSamplePolicy invalid_sample_policy{InvalidSamplePolicy::HoldState};  ///< Recovery-history policy.
            Filter filter{};                                                            ///< Filter instance.
            Rate rate{};                                                                ///< Rate limiter instance.
            Shaper shape{};                                                             ///< Response shaper.
        };

        /** @brief Public state produced by update(). */
        struct State
        {
            float raw01{0.f};      ///< Latest retained raw reading normalized to [0,1].
            float calib01{0.f};    ///< Latest retained calibrated value in [0,1].
            float centered{0.f};   ///< Latest retained final value in [-1,1].
            uint32_t t_ms{0};      ///< Timestamp of the latest update attempt.
            SampleStatus status{}; ///< Validity, acquisition time, sequence, and quality.
        };

        using Frame = State; ///< POD-style latest-state frame.

        LinearPot() = default;

        /** @brief Construct with configuration. */
        explicit LinearPot(const Config &c) : cfg_(c) { precompute_(); }

        /** @brief Inject a custom millisecond time source. */
        void setTimeSource(TimeFn fn) noexcept { time_fn_ = fn; }

        /** @brief Update using the configured/default time source. */
        void update() noexcept { update(detail::time_now_ms(time_fn_)); }

        /** @brief Update using an external timestamp with wrap-safe internal dt. */
        void update(uint32_t now_ms) noexcept { update(now_ms, dt_.step(now_ms)); }

        /**
         * @brief Acquire and process one sample.
         * @param now_ms Current timestamp in milliseconds.
         * @param dt_s Elapsed time in seconds.
         */
        void update(uint32_t now_ms, float dt_s) noexcept
        {
            st_.t_ms = now_ms;

            const bool calibration_ok = detail::calibration_valid(cfg_.calib, adc_full_scale_, cfg_.min_calibration_span);
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

            const RawSample sample = detail::acquire(cfg_.reader, adc_full_scale_);
            if (!sample.valid)
            {
                fail_(sample.error, calibration_ok, quality);
                return;
            }

            // Keep each processing stage independent. The filter owns filtered
            // history; the rate limiter owns only the final output history.
            const float raw01 = adc_full_scale_inv_ * static_cast<float>(sample.raw);
            const float v01 = detail::map_raw_to_01(sample.raw, cfg_.calib, adc_full_scale_, calibration_ok);
            const float shaped = cfg_.shape(detail::to_centered(v01));
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

            st_.raw01 = clamp01(raw01);
            st_.calib01 = clamp01(v01);
            st_.centered = clamp11(output);
            detail::mark_success(st_.status, now_ms, calibration_ok, quality);
        }

        /** @brief Get the latest state. */
        POTIO_NODISCARD const State &state() const noexcept { return st_; }

        /** @brief Get a copy of the latest state. */
        POTIO_NODISCARD Frame frame() const noexcept { return st_; }

        /** @brief True when the latest update produced fresh valid data. */
        POTIO_NODISCARD bool valid() const noexcept { return st_.status.valid; }

        /** @brief Get the calibrated [0,1] value. */
        POTIO_NODISCARD float calib01() const noexcept { return st_.calib01; }

        /** @brief Get the final centered [-1,1] value. */
        POTIO_NODISCARD float centered() const noexcept { return st_.centered; }

        /**
         * @brief Replace configuration.
         * @param c New configuration bundle.
         * @param reset_state Clear public state as well as processing history.
         *
         * @note Calibration changes always reset processing history so an old
         * filter state is never mixed with a new coordinate system.
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
                reset_processing_();
        }

        /** @brief Clear public state, timing, and processing history. */
        void resetState() noexcept
        {
            st_ = State{};
            dt_ = detail::DtState{};
            reset_processing_();
        }

        /** @brief Get the reader full-scale used for normalization. */
        POTIO_NODISCARD int fullScale() const noexcept { return adc_full_scale_; }

    private:
        void precompute_() noexcept
        {
            const int declared = detail::FullScale<Reader>::value;
            full_scale_valid_ = declared > 0;
            adc_full_scale_ = full_scale_valid_ ? declared : 1;
            adc_full_scale_inv_ = 1.0f / static_cast<float>(adc_full_scale_);
        }

        ReadError validate_config_() const noexcept
        {
            if (!full_scale_valid_ || cfg_.min_calibration_span == 0u)
                return ReadError::InvalidConfiguration;
            if (!detail::valid_calibration_policy(cfg_.calibration_policy) ||
                !detail::valid_invalid_sample_policy(cfg_.invalid_sample_policy))
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.max_dt_s) || cfg_.max_dt_s <= 0.0f)
                return ReadError::InvalidConfiguration;
            if (!detail::policy_valid(cfg_.filter) || !detail::policy_valid(cfg_.rate) || !detail::policy_valid(cfg_.shape))
                return ReadError::InvalidConfiguration;
            return ReadError::None;
        }

        void fail_(ReadError error, bool calibration_ok, uint16_t quality) noexcept
        {
            detail::mark_failure(st_.status, error, calibration_ok, quality);
            if (cfg_.invalid_sample_policy == InvalidSamplePolicy::ResetProcessing)
                reset_processing_();
        }

        void reset_processing_() noexcept
        {
            filtered_centered_ = 0.0f;
            has_processing_sample_ = false;
        }

        Config cfg_{}; ///< Active configuration.
        State st_{};   ///< Public state storage.

        int adc_full_scale_{POTIO_DEFAULT_FULLSCALE};
        float adc_full_scale_inv_{1.f / static_cast<float>(POTIO_DEFAULT_FULLSCALE)};
        bool full_scale_valid_{true};

        TimeFn time_fn_{nullptr};
        detail::DtState dt_{};
        float filtered_centered_{0.0f};
        bool has_processing_sample_{false};
    };

} ///< namespace PotIO
