/**
 * MIT License
 *
 * @brief Two-axis joystick with explicit validity, calibration, geometry, deadzones, shaping, filtering, and slew limiting.
 *
 * @file PotIO_Joystick2D.h
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
     * @brief Two-axis joystick with calibrated output in [-1,1].
     *
     * @details
     * Both axes form one acquisition. If either reader fails, neither axis
     * advances its processing history. The last good X/Y output is retained and
     * State::status.valid becomes false.
     */
    template <typename ReadX = ArduinoAnalogRead, typename ReadY = ArduinoAnalogRead,
              typename FilterX = EMAFilter, typename FilterY = EMAFilter,
              typename RateX = NoRateLimit, typename RateY = NoRateLimit,
              typename ShapeX = ShapeIdentity, typename ShapeY = ShapeIdentity,
              bool ComputeMag = true, bool ComputeAngle = true>
    class Joystick2D
    {
    public:
        /** @brief Configuration bundle for Joystick2D. */
        struct Config
        {
            ReadX readX{};                                                              ///< X-axis reader.
            ReadY readY{};                                                              ///< Y-axis reader.
            PotCalib calX{};                                                            ///< X-axis calibration.
            PotCalib calY{};                                                            ///< Y-axis calibration.
            CalibrationPolicy calibration_policy{CalibrationPolicy::PermissiveDefault}; ///< Calibration failure policy.
            uint16_t min_calibration_span{1u};                                          ///< Minimum raw span each side of center.
            Deadzone deadzone{Deadzone::RadialScaled};                                  ///< Deadzone strategy.
            float deadzone_size{0.12f};                                                 ///< Deadzone fraction in [0,0.95].
            JoystickGeometry geometry{JoystickGeometry::Square};                        ///< Output geometry after deadzone processing.
            float angle_min_magnitude{0.001f};                                          ///< Magnitude below which angle is semantically invalid.
            float max_dt_s{0.5f};                                                       ///< Largest accepted processing gap in seconds.
            InvalidSamplePolicy invalid_sample_policy{InvalidSamplePolicy::HoldState};  ///< Recovery-history policy.
            FilterX fx{};                                                               ///< X filter.
            FilterY fy{};                                                               ///< Y filter.
            RateX rx{};                                                                 ///< X rate limiter.
            RateY ry{};                                                                 ///< Y rate limiter.
            ShapeX sx{};                                                                ///< X response shaper.
            ShapeY sy{};                                                                ///< Y response shaper.
        };

        /** @brief Public state produced by update(). */
        struct State
        {
            float x{0.f};            ///< Retained X output in [-1,1].
            float y{0.f};            ///< Retained Y output in [-1,1].
            float mag{0.f};          ///< Retained magnitude in [0,1] when enabled.
            float angle{0.f};        ///< Retained angle in radians when enabled and meaningful.
            bool angle_valid{false}; ///< True when angle represents a non-neutral vector.
            uint32_t t_ms{0};        ///< Timestamp of the latest update attempt.
            SampleStatus status{};   ///< Validity, sample timestamp, sequence, and quality.
        };

        using Frame = State; ///< POD-style latest-state frame.

        Joystick2D() = default;

        /** @brief Construct with configuration. */
        explicit Joystick2D(const Config &c) : cfg_(c) { precompute_(); }

        /** @brief Inject a custom millisecond time source. */
        void setTimeSource(TimeFn fn) noexcept { time_fn_ = fn; }

        /** @brief Update using the configured/default time source. */
        void update() noexcept { update(detail::time_now_ms(time_fn_)); }

        /** @brief Update using an external timestamp with wrap-safe internal dt. */
        void update(uint32_t now_ms) noexcept { update(now_ms, dt_.step(now_ms)); }

        /** @brief Acquire and process one two-axis sample. */
        void update(uint32_t now_ms, float dt_s) noexcept
        {
            st_.t_ms = now_ms;

            const bool cal_x_ok = detail::calibration_valid(cfg_.calX, fs_x_, cfg_.min_calibration_span);
            const bool cal_y_ok = detail::calibration_valid(cfg_.calY, fs_y_, cfg_.min_calibration_span);
            const bool calibration_ok = cal_x_ok && cal_y_ok;
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

            const RawSample sample_x = detail::acquire(cfg_.readX, fs_x_);
            const RawSample sample_y = detail::acquire(cfg_.readY, fs_y_);
            if (!sample_x.valid || !sample_y.valid)
            {
                fail_(!sample_x.valid ? sample_x.error : sample_y.error, calibration_ok, quality);
                return;
            }

            float x = detail::to_centered(detail::map_raw_to_01(sample_x.raw, cfg_.calX, fs_x_, cal_x_ok));
            float y = detail::to_centered(detail::map_raw_to_01(sample_y.raw, cfg_.calY, fs_y_, cal_y_ok));

            apply_deadzone_(x, y);
            apply_geometry_(x, y);

            const float shaped_x = cfg_.sx(clamp11(x));
            const float shaped_y = cfg_.sy(clamp11(y));
            if (!detail::finite_float(shaped_x) || !detail::finite_float(shaped_y))
            {
                fail_(ReadError::InvalidConfiguration, calibration_ok, quality);
                return;
            }

            float filtered_x = shaped_x;
            float filtered_y = shaped_y;
            float output_x = shaped_x;
            float output_y = shaped_y;

            if (has_processing_sample_)
            {
                filtered_x = cfg_.fx(filtered_x_, shaped_x);
                filtered_y = cfg_.fy(filtered_y_, shaped_y);
                if (!detail::finite_float(filtered_x) || !detail::finite_float(filtered_y))
                {
                    fail_(ReadError::InvalidConfiguration, calibration_ok, quality);
                    return;
                }
                output_x = cfg_.rx(st_.x, filtered_x, dt_s);
                output_y = cfg_.ry(st_.y, filtered_y, dt_s);
                if (!detail::finite_float(output_x) || !detail::finite_float(output_y))
                {
                    fail_(ReadError::InvalidConfiguration, calibration_ok, quality);
                    return;
                }
            }

            filtered_x_ = clamp11(filtered_x);
            filtered_y_ = clamp11(filtered_y);
            has_processing_sample_ = true;

            st_.x = clamp11(output_x);
            st_.y = clamp11(output_y);

            const float vector_mag = sqrt(st_.x * st_.x + st_.y * st_.y);
            st_.mag = ComputeMag ? ((vector_mag > 1.f) ? 1.f : vector_mag) : 0.f;

            if (ComputeAngle && vector_mag > cfg_.angle_min_magnitude)
            {
                st_.angle = atan2(st_.y, st_.x);
                st_.angle_valid = true;
            }
            else
            {
                st_.angle = 0.f;
                st_.angle_valid = false;
            }

            detail::mark_success(st_.status, now_ms, calibration_ok, quality);
        }

        /** @brief Get the latest state. */
        POTIO_NODISCARD const State &state() const noexcept { return st_; }

        /** @brief Get a copy of the latest state. */
        POTIO_NODISCARD Frame frame() const noexcept { return st_; }

        /** @brief True when the latest update produced fresh valid data. */
        POTIO_NODISCARD bool valid() const noexcept { return st_.status.valid; }

        /** @brief Get latest X-axis value. */
        POTIO_NODISCARD float x() const noexcept { return st_.x; }

        /** @brief Get latest Y-axis value. */
        POTIO_NODISCARD float y() const noexcept { return st_.y; }

        /** @brief Get latest radial magnitude. */
        POTIO_NODISCARD float magnitude() const noexcept { return st_.mag; }

        /** @brief True when angleRad()/angleDeg() is meaningful. */
        POTIO_NODISCARD bool angleValid() const noexcept { return st_.angle_valid; }

        /** @brief Get latest angle in radians; check angleValid() at neutral. */
        POTIO_NODISCARD float angleRad() const noexcept { return st_.angle; }

        /** @brief Get latest angle in degrees; check angleValid() at neutral. */
        POTIO_NODISCARD float angleDeg() const noexcept { return rad2deg(st_.angle); }

        /**
         * @brief Replace configuration.
         * @param c New configuration bundle.
         * @param reset_state Clear public state as well as processing history.
         */
        void setConfig(const Config &c, bool reset_state = false) noexcept
        {
            const bool calibration_changed = !detail::calib_equal(cfg_.calX, c.calX) ||
                                             !detail::calib_equal(cfg_.calY, c.calY) ||
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

        /** @brief Get X reader full-scale. */
        POTIO_NODISCARD int fullScaleX() const noexcept { return fs_x_; }

        /** @brief Get Y reader full-scale. */
        POTIO_NODISCARD int fullScaleY() const noexcept { return fs_y_; }

    private:
        void precompute_() noexcept
        {
            const int declared_x = detail::FullScale<ReadX>::value;
            const int declared_y = detail::FullScale<ReadY>::value;
            full_scale_valid_ = declared_x > 0 && declared_y > 0;
            fs_x_ = (declared_x > 0) ? declared_x : 1;
            fs_y_ = (declared_y > 0) ? declared_y : 1;
        }

        ReadError validate_config_() const noexcept
        {
            if (!full_scale_valid_ || cfg_.min_calibration_span == 0u)
                return ReadError::InvalidConfiguration;
            if (!detail::valid_calibration_policy(cfg_.calibration_policy) ||
                !detail::valid_invalid_sample_policy(cfg_.invalid_sample_policy) ||
                !detail::valid_deadzone(cfg_.deadzone) ||
                !detail::valid_geometry(cfg_.geometry))
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.deadzone_size) || cfg_.deadzone_size < 0.f || cfg_.deadzone_size > 0.95f)
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.angle_min_magnitude) || cfg_.angle_min_magnitude < 0.f || cfg_.angle_min_magnitude > 1.f)
                return ReadError::InvalidConfiguration;
            if (!detail::finite_float(cfg_.max_dt_s) || cfg_.max_dt_s <= 0.f)
                return ReadError::InvalidConfiguration;
            if (!detail::policy_valid(cfg_.fx) || !detail::policy_valid(cfg_.fy) ||
                !detail::policy_valid(cfg_.rx) || !detail::policy_valid(cfg_.ry) ||
                !detail::policy_valid(cfg_.sx) || !detail::policy_valid(cfg_.sy))
                return ReadError::InvalidConfiguration;
            return ReadError::None;
        }

        void apply_deadzone_(float &x, float &y) const noexcept
        {
            const float dz = cfg_.deadzone_size;

            if (cfg_.deadzone == Deadzone::None)
                return;

            if (cfg_.deadzone == Deadzone::Axial || cfg_.deadzone == Deadzone::AxialScaled)
            {
                x = apply_axial_(x, dz, cfg_.deadzone == Deadzone::AxialScaled);
                y = apply_axial_(y, dz, cfg_.deadzone == Deadzone::AxialScaled);
                return;
            }

            const float m = sqrt(x * x + y * y);
            if (m <= dz)
            {
                x = 0.f;
                y = 0.f;
                return;
            }

            if (cfg_.deadzone == Deadzone::RadialScaled && m > kEps)
            {
                // Only scale the unit-circle portion. Square-corner vectors with
                // magnitude > 1 are never amplified by the radial remap.
                const float bounded_m = (m > 1.f) ? 1.f : m;
                const float scaled_m = (bounded_m - dz) / (1.f - dz);
                const float scale = (m > 1.f) ? 1.f : (scaled_m / m);
                x *= scale;
                y *= scale;
            }
        }

        static float apply_axial_(float value, float dz, bool scaled) noexcept
        {
            const float a = fabs(value);
            if (a <= dz)
                return 0.f;
            if (!scaled)
                return value;
            const float remapped = (a - dz) / (1.f - dz);
            return (value < 0.f) ? -remapped : remapped;
        }

        void apply_geometry_(float &x, float &y) const noexcept
        {
            x = clamp11(x);
            y = clamp11(y);

            if (cfg_.geometry == JoystickGeometry::Square)
                return;

            if (cfg_.geometry == JoystickGeometry::MagnitudeClamp)
            {
                const float m = sqrt(x * x + y * y);
                if (m > 1.f && m > kEps)
                {
                    x /= m;
                    y /= m;
                }
                return;
            }

            if (cfg_.geometry == JoystickGeometry::SquareToCircle)
            {
                // Smooth square-to-circle mapping. Independent per-axis calibration
                // can already account for unequal X/Y spans before this transform.
                const float x0 = x;
                const float y0 = y;
                const float x_term = 1.f - 0.5f * y0 * y0;
                const float y_term = 1.f - 0.5f * x0 * x0;
                x = x0 * sqrt((x_term > 0.f) ? x_term : 0.f);
                y = y0 * sqrt((y_term > 0.f) ? y_term : 0.f);
            }
        }

        void fail_(ReadError error, bool calibration_ok, uint16_t quality) noexcept
        {
            detail::mark_failure(st_.status, error, calibration_ok, quality);
            st_.angle_valid = false;
            if (cfg_.invalid_sample_policy == InvalidSamplePolicy::ResetProcessing)
                reset_processing_();
        }

        void reset_processing_() noexcept
        {
            filtered_x_ = 0.f;
            filtered_y_ = 0.f;
            has_processing_sample_ = false;
        }

        Config cfg_{};
        State st_{};
        int fs_x_{POTIO_DEFAULT_FULLSCALE};
        int fs_y_{POTIO_DEFAULT_FULLSCALE};
        bool full_scale_valid_{true};
        TimeFn time_fn_{nullptr};
        detail::DtState dt_{};
        float filtered_x_{0.f};
        float filtered_y_{0.f};
        bool has_processing_sample_{false};
    };

} ///< namespace PotIO
