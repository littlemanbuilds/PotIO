/**
 * MIT License
 *
 * @brief Discrete-step potentiometer with explicit validity, hysteresis, and lossless change sequencing.
 *
 * @file PotIO_SteppedPot.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "PotIO_Detail.h"
#include "PotIO_Filters.h"
#include "PotIO_Arduino.h"

namespace PotIO
{
    /**
     * @brief Map an analog potentiometer to N stable steps with hysteresis.
     *
     * @details
     * `changed` remains a convenient one-update pulse. `change_sequence` is the
     * durable companion for latest-state transports: consumers can compare the
     * counter and detect that one or more transitions happened even if they did
     * not observe the pulse itself.
     */
    template <size_t Steps,
              typename Reader = ArduinoAnalogRead,
              typename Filter = NoFilter>
    class SteppedPot
    {
        static_assert(Steps >= 2u, "SteppedPot<Steps>: Steps must be >= 2.");
        static_assert(Steps <= 255u, "SteppedPot<Steps>: Steps must fit in uint8_t.");

    public:
        /** @brief Configuration bundle for SteppedPot. */
        struct Config
        {
            Reader reader{};                                                            ///< ADC reader.
            PotCalib calib{};                                                           ///< Min/center/max calibration.
            CalibrationPolicy calibration_policy{CalibrationPolicy::PermissiveDefault}; ///< Calibration failure policy.
            uint16_t min_calibration_span{1u};                                          ///< Minimum raw counts on each side of center.
            Filter filter{};                                                            ///< Optional normalized filter.
            float hysteresis{0.04f};                                                    ///< Fraction of one step width used as hold band [0,0.49].
            float max_dt_s{0.5f};                                                       ///< Largest accepted update gap in seconds.
            InvalidSamplePolicy invalid_sample_policy{InvalidSamplePolicy::HoldState};  ///< Filter recovery policy.
        };

        /** @brief Public state produced by update(). */
        struct State
        {
            uint16_t raw{0};             ///< Latest retained raw ADC sample.
            float v01{0.f};              ///< Latest retained filtered normalized reading.
            uint8_t step{0};             ///< Current retained step in [0,Steps-1].
            bool changed{false};         ///< True only when this valid update changed step.
            uint32_t change_sequence{0}; ///< Monotonic step-change counter; wraps naturally.
            uint32_t t_ms{0};            ///< Timestamp of the latest update attempt.
            SampleStatus status{};       ///< Validity, sample timestamp, sequence, and quality.
        };

        using Frame = State; ///< POD-style latest-state frame.

        /**
         * @brief Construct an unseeded device with default configuration and cache its reader range.
         */
        SteppedPot() noexcept(noexcept(Config{})) { precompute_(); }

        /**
         * @brief Copy configuration into an unseeded device.
         *
         * @param c Configuration; validity is checked during update().
         */
        explicit SteppedPot(const Config &c) : cfg_(c) { precompute_(); }

        /**
         * @brief Select the clock used by update().
         *
         * @param fn Millisecond clock, or nullptr for Arduino millis() (zero on a host).
         */
        void setTimeSource(TimeFn fn) noexcept { time_fn_ = fn; }

        /** @brief Update using the configured/default time source. */
        void update() noexcept { update(detail::time_now_ms(time_fn_)); }

        /**
         * @brief Update using elapsed time since the preceding timestamp-only call.
         *
         * @param now_ms Current millisecond tick; first call uses zero elapsed time.
         * @note The attempt clock advances even on failure. Explicit-dt calls do not advance it.
         */
        void update(uint32_t now_ms) noexcept { update(now_ms, dt_.step(now_ms)); }

        /**
         * @brief Acquire and quantize one sample.
         *
         * @param now_ms Current millisecond timestamp.
         * @param dt_s Elapsed processing seconds, finite and in [0,max_dt_s].
         * @note Does not advance the internal timestamp-only clock. Failure retains output values.
         */
        void update(uint32_t now_ms, float dt_s) noexcept
        {
            st_.t_ms = now_ms;
            st_.changed = false;

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

            const float v01_in = detail::map_raw_to_01(sample.raw, cfg_.calib, fs_, calibration_ok);
            float v01 = v01_in;
            if (has_processing_sample_)
                v01 = cfg_.filter(filtered_v01_, v01_in);
            if (!detail::finite_float(v01))
            {
                fail_(ReadError::InvalidConfiguration, calibration_ok, quality);
                return;
            }

            filtered_v01_ = clamp01(v01);
            has_processing_sample_ = true;

            const uint8_t previous = st_.step;
            const uint8_t next = has_step_ ? quantize_(filtered_v01_, previous)
                                           : quantize_first_(filtered_v01_);
            const bool changed = has_step_ && next != previous;
            has_step_ = true;

            st_.raw = static_cast<uint16_t>(sample.raw);
            st_.v01 = filtered_v01_;
            st_.step = next;
            st_.changed = changed;
            if (changed)
                ++st_.change_sequence;
            detail::mark_success(st_.status, now_ms, calibration_ok, quality);
        }

        /**
         * @brief Get a reference to the latest state.
         *
         * @return Internal state, changed by subsequent updates or resets; check status.valid for freshness.
         */
        POTIO_NODISCARD const State &state() const noexcept { return st_; }

        /**
         * @brief Copy the latest state for application-owned storage.
         *
         * @return State snapshot including validity and retained values; concurrent access requires synchronization.
         */
        POTIO_NODISCARD Frame frame() const noexcept { return st_; }

        /**
         * @brief Check whether the latest update produced usable data.
         *
         * @return True for a successful latest update, false initially or after failure.
         */
        POTIO_NODISCARD bool valid() const noexcept { return st_.status.valid; }

        /**
         * @brief Get the durable step-change counter.
         *
         * @return Wrapping count of accepted step transitions; first seeding is not a transition.
         */
        POTIO_NODISCARD uint32_t changeSequence() const noexcept { return st_.change_sequence; }

        /**
         * @brief Replace configuration.
         *
         * @param c New configuration bundle.
         * @param reset_state Clear public state, current step, and filter history.
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
                has_step_ = false;
            }
        }

        /** @brief Clear public state, timing, filter history, and step seed. */
        void resetState() noexcept
        {
            st_ = State{};
            dt_ = detail::DtState{};
            has_step_ = false;
            reset_processing_();
        }

        /**
         * @brief Get effective reader full-scale.
         *
         * @return Positive reader maximum, or one when the declaration is invalid; update() rejects that configuration.
         */
        POTIO_NODISCARD int fullScale() const noexcept { return fs_; }

    private:
        /**
         * @brief Cache the reader range, substituting one for an invalid declaration.
         * Invalid declarations still cause update() to reject the configuration.
         */
        void precompute_() noexcept
        {
            const int declared = detail::FullScale<Reader>::value;
            full_scale_valid_ = declared > 0;
            fs_ = full_scale_valid_ ? declared : 1;
        }

        /**
         * @brief Validate device settings and available policy validation hooks.
         *
         * @return InvalidConfiguration for unusable settings, otherwise None; calibration is checked separately.
         */
        ReadError validate_config_() const noexcept
        {
            if (!full_scale_valid_ || cfg_.min_calibration_span == 0u)
                return ReadError::InvalidConfiguration;
            if (!detail::valid_calibration_policy(cfg_.calibration_policy) ||
                !detail::valid_invalid_sample_policy(cfg_.invalid_sample_policy))
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.hysteresis) || cfg_.hysteresis < 0.f || cfg_.hysteresis > 0.49f)
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.max_dt_s) || cfg_.max_dt_s <= 0.f)
                return ReadError::InvalidConfiguration;
            if (!detail::policy_valid(cfg_.filter))
                return ReadError::InvalidConfiguration;
            return ReadError::None;
        }

        /**
         * @brief Get the uniform bin width in normalized coordinates.
         *
         * @return 1/Steps, with Steps constrained at compile time to [2,255].
         */
        static float step_width_() noexcept
        {
            return 1.0f / static_cast<float>(Steps);
        }

        /**
         * @brief Select a bin without a previous step or hysteresis.
         *
         * @param v01 Finite normalized input.
         * @return Bin index clamped to [0,Steps-1], including the full-scale endpoint.
         */
        uint8_t quantize_first_(float v01) const noexcept
        {
            const float w = step_width_();
            const int idx = static_cast<int>(floor(v01 / w));
            if (idx < 0)
                return 0u;
            if (idx >= static_cast<int>(Steps))
                return static_cast<uint8_t>(Steps - 1u);
            return static_cast<uint8_t>(idx);
        }

        /**
         * @brief Retain the current bin inside its inclusive hysteresis band.
         *
         * @param v01 Finite normalized input.
         * @param current Previous accepted step in [0,Steps-1].
         * @return Current step within the hold band, otherwise the direct destination bin.
         */
        uint8_t quantize_(float v01, uint8_t current) const noexcept
        {
            const float w = step_width_();
            const float band = cfg_.hysteresis * w;
            const float lo = static_cast<float>(current) * w - band;
            const float hi = static_cast<float>(current + 1u) * w + band;
            if (v01 >= lo && v01 <= hi)
                return current;
            return quantize_first_(v01);
        }

        /**
         * @brief Mark this attempt invalid and apply the selected history recovery policy.
         *
         * @param error Failure reason.
         * @param calibration_ok Whether configured calibration is usable.
         * @param quality Quality flags for this attempt.
         */
        void fail_(ReadError error, bool calibration_ok, uint16_t quality) noexcept
        {
            detail::mark_failure(st_.status, error, calibration_ok, quality);
            st_.changed = false;
            if (cfg_.invalid_sample_policy == InvalidSamplePolicy::ResetProcessing)
                reset_processing_();
        }

        /**
         * @brief Discard filter history so the next accepted input seeds processing.
         * Public output and the attempt clock are retained.
         */
        void reset_processing_() noexcept
        {
            filtered_v01_ = 0.f;
            has_processing_sample_ = false;
        }

        Config cfg_{};
        State st_{};
        int fs_{POTIO_DEFAULT_FULLSCALE};
        bool full_scale_valid_{true};
        TimeFn time_fn_{nullptr};
        detail::DtState dt_{};
        float filtered_v01_{0.f};
        bool has_processing_sample_{false};
        bool has_step_{false};
    };

} ///< namespace PotIO
