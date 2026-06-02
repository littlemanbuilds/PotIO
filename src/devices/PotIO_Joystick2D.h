/**
 * MIT License
 *
 * @brief 2D joystick with calibration, deadzones, shaping, filtering, and slew limiting.
 *
 * @file PotIO_Joystick2D.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include <PotIO_Detail.h>
#include <PotIO_Filters.h>
#include <PotIO_RateLimit.h>
#include <PotIO_Shaping.h>
#include <PotIO_Arduino.h>

namespace PotIO
{
    /**
     * @brief 2D joystick device: calibrated X/Y in [-1,1], optional magnitude and angle.
     *
     * @details
     * This class is intentionally single-threaded: call update() from one context.
     * For cross-task/cross-core consumers, publish a POD frame using SnapshotBus.
     *
     * @tparam ReadX Reader functor for X axis (may define `static constexpr int kFullScale`).
     * @tparam ReadY Reader functor for Y axis (may define `static constexpr int kFullScale`).
     * @tparam FilterX Filter for X samples (e.g., EMAFilter).
     * @tparam FilterY Filter for Y samples (e.g., EMAFilter).
     * @tparam RateX Rate limiter for X (e.g., NoRateLimit or SlewRate).
     * @tparam RateY Rate limiter for Y.
     * @tparam ShapeX Shaper for X (e.g., ShapeIdentity).
     * @tparam ShapeY Shaper for Y.
     * @tparam ComputeMag If true, compute magnitude in [0,1].
     * @tparam ComputeAngle If true, compute angle (radians) via atan2(y,x).
     */
    template <typename ReadX = ArduinoAnalogRead, typename ReadY = ArduinoAnalogRead,
              typename FilterX = EMAFilter, typename FilterY = EMAFilter,
              typename RateX = NoRateLimit, typename RateY = NoRateLimit,
              typename ShapeX = ShapeIdentity, typename ShapeY = ShapeIdentity,
              bool ComputeMag = true, bool ComputeAngle = true>
    class Joystick2D
    {
    public:
        /**
         * @brief Configuration bundle for Joystick2D.
         */
        struct Config
        {
            ReadX readX{};                             ///< X reader.
            ReadY readY{};                             ///< Y reader.
            PotCalib calX{};                           ///< X calibration.
            PotCalib calY{};                           ///< Y calibration.
            Deadzone deadzone{Deadzone::RadialScaled}; ///< Deadzone strategy.
            float deadzone_size{0.12f};                ///< Deadzone fraction (0..1).
            FilterX fx{};                              ///< X filter.
            FilterY fy{};                              ///< Y filter.
            RateX rx{};                                ///< X rate limiter.
            RateY ry{};                                ///< Y rate limiter.
            ShapeX sx{};                               ///< X shaper.
            ShapeY sy{};                               ///< Y shaper.
        };

        /**
         * @brief Public state produced by update().
         */
        struct State
        {
            float x{0.f};       ///< X in [-1,1].
            float y{0.f};       ///< Y in [-1,1].
            float mag{0.f};     ///< Magnitude in [0,1] (if enabled).
            float angle{0.f};   ///< Angle in radians (if enabled).
            uint32_t t_ms{0};   ///< Timestamp in milliseconds.
        };

        /**
         * @brief POD snapshot suitable for publishing via SnapshotBus.
         */
        using Frame = State;

        Joystick2D() = default;

        /**
         * @brief Construct with configuration.
         * @param c Configuration bundle.
         */
        explicit Joystick2D(const Config &c) : cfg_(c) { precompute_(); }

        /**
         * @brief Inject a custom time source (milliseconds). Pass nullptr to revert to default (Arduino::millis()).
         * @param fn Optional function returning current milliseconds.
         */
        void setTimeSource(TimeFn fn) noexcept { time_fn_ = fn; }

        /**
         * @brief Update using internal time source (no arguments).
         */
        void update() noexcept { update(detail::time_now_ms(time_fn_)); }

        /**
         * @brief Update using external timestamp; dt computed internally (wrap-safe).
         * @param now_ms Current timestamp in milliseconds.
         */
        void update(uint32_t now_ms) noexcept
        {
            const float dt_s = dt_.step(now_ms);
            update(now_ms, dt_s);
        }

        /**
         * @brief Update using explicit time step.
         * @param now_ms Current timestamp in milliseconds.
         * @param dt_s Elapsed time in seconds for filters and rate limits.
         */
        void update(uint32_t now_ms, float dt_s) noexcept
        {
            // ---- Raw -> [0..1] ---- //
            const int rx = detail::clamp_raw(cfg_.readX(), fs_x_);
            const int ry = detail::clamp_raw(cfg_.readY(), fs_y_);

            const float x01 = detail::map_raw_to_01(rx, cfg_.calX, fs_x_);
            const float y01 = detail::map_raw_to_01(ry, cfg_.calY, fs_y_);

            // ---- Center -> [-1..1] ---- //
            float x = detail::to_centered(x01);
            float y = detail::to_centered(y01);

            // ---- Deadzone ---- //
            const float dz = (cfg_.deadzone_size < 0.f) ? 0.f : (cfg_.deadzone_size > 0.95f ? 0.95f : cfg_.deadzone_size);

            if (cfg_.deadzone == Deadzone::Axial)
            {
                if (fabs(x) < dz)
                    x = 0.f;
                if (fabs(y) < dz)
                    y = 0.f;
            }
            else if (cfg_.deadzone == Deadzone::Radial || cfg_.deadzone == Deadzone::RadialScaled)
            {
                const float m = sqrt(x * x + y * y);
                if (m < dz)
                {
                    x = 0.f;
                    y = 0.f;
                }
                else if (cfg_.deadzone == Deadzone::RadialScaled && m > kEps)
                {
                    // Scale magnitude from [dz..1] -> [0..1] while preserving direction.
                    const float scaled = (m - dz) / (1.f - dz);
                    const float s = scaled / m;
                    x *= s;
                    y *= s;
                }
            }

            // ---- Shape/Filter/Rate (per axis) ---- //
            x = cfg_.sx(clamp11(x));
            y = cfg_.sy(clamp11(y));

            if (has_sample_)
            {
                x = cfg_.fx(st_.x, x);
                y = cfg_.fy(st_.y, y);

                x = cfg_.rx(st_.x, x, dt_s);
                y = cfg_.ry(st_.y, y, dt_s);
            }
            else
            {
                // First sample seeds both axes together to avoid a diagonal ramp from (0,0).
                has_sample_ = true;
            }

            st_.x = clamp11(x);
            st_.y = clamp11(y);

            if (ComputeMag)
            {
                const float m = sqrt(st_.x * st_.x + st_.y * st_.y);
                st_.mag = (m > 1.f) ? 1.f : m;
            }
            else
            {
                st_.mag = 0.f;
            }

            if (ComputeAngle)
            {
                st_.angle = atan2(st_.y, st_.x);
            }
            else
            {
                st_.angle = 0.f;
            }

            st_.t_ms = now_ms;
        }

        /**
         * @brief Get a const reference to the latest state.
         * @return Latest state storage owned by this object.
         */
        POTIO_NODISCARD const State &state() const noexcept { return st_; }

        /**
         * @brief Get a POD frame copy for latest-state publishing.
         * @return Copy of the latest state.
         */
        POTIO_NODISCARD Frame frame() const noexcept { return st_; }

        /**
         * @brief Get latest X-axis value.
         * @return X in [-1,1].
         */
        POTIO_NODISCARD float x() const noexcept { return st_.x; }

        /**
         * @brief Get latest Y-axis value.
         * @return Y in [-1,1].
         */
        POTIO_NODISCARD float y() const noexcept { return st_.y; }

        /**
         * @brief Get latest joystick magnitude.
         * @return Magnitude in [0,1], or 0 when magnitude computation is disabled.
         */
        POTIO_NODISCARD float magnitude() const noexcept { return st_.mag; }

        /**
         * @brief Get latest joystick angle in radians.
         * @return Angle from +X via atan2(y,x), or 0 when angle computation is disabled.
         */
        POTIO_NODISCARD float angleRad() const noexcept { return st_.angle; }

        /**
         * @brief Get latest joystick angle in degrees.
         * @return Angle in degrees.
         */
        POTIO_NODISCARD float angleDeg() const noexcept { return rad2deg(st_.angle); }

        /**
         * @brief Replace configuration.
         *
         * @param c New configuration bundle.
         * @param reset_state If true, clear filter/rate history before the next update.
         */
        void setConfig(const Config &c, bool reset_state = false) noexcept
        {
            cfg_ = c;
            precompute_();
            if (reset_state)
                resetState();
        }

        /**
         * @brief Clear output history and timing state without changing configuration.
         */
        void resetState() noexcept
        {
            st_ = State{};
            dt_ = detail::DtState{};
            has_sample_ = false;
        }

        /**
         * @brief Get X reader full-scale.
         * @return Full-scale value in X reader units.
         */
        POTIO_NODISCARD int fullScaleX() const noexcept { return fs_x_; }

        /**
         * @brief Get Y reader full-scale.
         * @return Full-scale value in Y reader units.
         */
        POTIO_NODISCARD int fullScaleY() const noexcept { return fs_y_; }

    private:
        /**
         * @brief Precompute reader-dependent full-scale values for both axes.
         */
        void precompute_() noexcept
        {
            fs_x_ = detail::FullScale<ReadX>::value;
            fs_y_ = detail::FullScale<ReadY>::value;
            if (fs_x_ <= 0)
                fs_x_ = 1;
            if (fs_y_ <= 0)
                fs_y_ = 1;
        }

        Config cfg_{};
        State st_{};
        int fs_x_{POTIO_DEFAULT_FULLSCALE};
        int fs_y_{POTIO_DEFAULT_FULLSCALE};
        TimeFn time_fn_{nullptr};
        detail::DtState dt_{};
        bool has_sample_{false};
    };

} ///< namespace PotIO
