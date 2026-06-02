/**
 * MIT License
 *
 * @brief Endless rotary potentiometer: phase, turns, and unwrapped angle with filtering/shaping.
 *
 * @file PotIO_ContinuousPot.h
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
     * @brief Continuous (endless) potentiometer with wrap tracking and hysteresis.
     *
     * @details
     * The device maintains a signed turn counter by detecting when the phase crosses
     * the 0/1 boundary. A hysteresis band prevents false wrap toggles due to jitter.
     *
     * This class is intentionally single-threaded: call update() from one context.
     * For cross-task/cross-core consumers, publish a POD frame using SnapshotBus.
     *
     * @tparam Reader ADC reader functor (e.g., ArduinoAnalogRead). May define `static constexpr int kFullScale`.
     * @tparam Filter Sample filter (e.g., EMAFilter).
     * @tparam Rate Rate limiter (e.g., NoRateLimit or SlewRate).
     * @tparam Shaper Response shaper (e.g., ShapeIdentity).
     */
    template <typename Reader = ArduinoAnalogRead,
              typename Filter = EMAFilter,
              typename Rate = NoRateLimit,
              typename Shaper = ShapeIdentity>
    class ContinuousPot
    {
    public:
        /**
         * @brief Configuration bundle for ContinuousPot.
         */
        struct Config
        {
            Reader reader{};        ///< ADC reader functor.
            PotCalib calib{};       ///< Calibration (min/center/max); if invalid -> raw normalization.
            Filter filter{};        ///< Filter instance.
            Rate rate{};            ///< Rate limiter instance.
            Shaper shape{};         ///< Response shaper.
            float wrap_hyst{0.05f}; ///< Hysteresis near 0/1 to avoid false wrap toggles (fraction of [0,1]).
        };

        /**
         * @brief Public state produced by update().
         */
        struct State
        {
            uint16_t raw{0};       ///< Raw ADC sample in native units.
            float phase01{0.f};    ///< Normalized phase in [0,1).
            float centered{0.f};   ///< Centered [-1,1] after shaping/filter/rate.
            int32_t turns{0};      ///< Signed full turns accumulated since start.
            uint32_t t_ms{0};      ///< Timestamp in milliseconds.
        };

        /**
         * @brief POD snapshot suitable for publishing via SnapshotBus.
         */
        using Frame = State;

        ContinuousPot() = default;

        /**
         * @brief Construct with configuration.
         * @param c Configuration bundle.
         */
        explicit ContinuousPot(const Config &c) : cfg_(c) { precompute_(); }

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
            // 1) Raw read and clamp to ADC full scale.
            const int raw_i = detail::clamp_raw(cfg_.reader(), fs_);
            const float norm01 = detail::map_raw_to_01(raw_i, cfg_.calib, fs_);

            // 2) Accumulate wraps into signed turns.
            accumulate_wrap_(norm01);

            // 3) Center -> shape -> filter -> rate.
            float ctr = detail::to_centered(norm01);
            ctr = cfg_.shape(ctr);
            if (has_sample_)
            {
                ctr = cfg_.filter(st_.centered, ctr);
                ctr = cfg_.rate(st_.centered, ctr, dt_s);
            }
            else
            {
                // First sample seeds the shaped output; wrap tracking is seeded separately.
                has_sample_ = true;
            }
            ctr = clamp11(ctr);

            // 4) Publish.
            st_.raw = static_cast<uint16_t>(raw_i);
            st_.phase01 = norm01;
            st_.centered = ctr;
            st_.turns = turns_;
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
         * @brief Total turns since start (can be negative).
         * @return Signed full turns accumulated since reset/start.
         */
        POTIO_NODISCARD float turns_raw() const noexcept { return static_cast<float>(turns_); }

        /**
         * @brief Unwrapped angle in degrees: (turns + phase01) * 360.
         * @return Unwrapped angle in degrees.
         */
        POTIO_NODISCARD float degrees_raw() const noexcept { return (static_cast<float>(turns_) + st_.phase01) * 360.0f; }

        /**
         * @brief Current phase in [0,1).
         * @return Latest normalized phase.
         */
        POTIO_NODISCARD float phase01() const noexcept { return st_.phase01; }

        /**
         * @brief Current phase mapped to radians in [-π, +π].
         * @return Phase angle in radians.
         */
        POTIO_NODISCARD float phaseRad() const noexcept
        {
            // Map 0..1 to -π..+π with 0.5 -> 0
            return (st_.phase01 * 2.0f - 1.0f) * kPi;
        }

        /**
         * @brief Current phase mapped to degrees in [-180, +180].
         * @return Phase angle in degrees.
         */
        POTIO_NODISCARD float phaseDeg() const noexcept { return rad2deg(phaseRad()); }

        /**
         * @brief Unwrapped angle in radians since start.
         * @return Accumulated angle in radians.
         */
        POTIO_NODISCARD float angleUnwrappedRad() const noexcept
        {
            return (static_cast<float>(turns_) + st_.phase01) * (2.0f * kPi);
        }

        /**
         * @brief Unwrapped angle in degrees since start.
         * @return Accumulated angle in degrees.
         */
        POTIO_NODISCARD float angleUnwrappedDeg() const noexcept { return degrees_raw(); }

        /**
         * @brief Replace configuration.
         *
         * @param c New configuration bundle.
         * @param reset_state If true, clear turns, filter/rate history, timing, and wrap seed.
         */
        void setConfig(const Config &c, bool reset_state = false) noexcept
        {
            cfg_ = c;
            precompute_();
            if (reset_state)
                resetState();
        }

        /**
         * @brief Reset accumulated turns to zero.
         *
         * @note The current phase remains unchanged; future wraps accumulate from zero.
         */
        void resetTurns() noexcept
        {
            turns_ = 0;
            st_.turns = 0;
        }

        /**
         * @brief Clear public state, accumulated turns, timing state, and wrap history.
         */
        void resetState() noexcept
        {
            st_ = State{};
            dt_ = detail::DtState{};
            last_phase01_ = 0.f;
            turns_ = 0;
            has_sample_ = false;
            has_last_phase_ = false;
        }

        /**
         * @brief Get effective ADC full-scale (Reader::kFullScale or default).
         * @return Full-scale value in raw reader units.
         */
        POTIO_NODISCARD int fullScale() const noexcept { return fs_; }

    private:
        /**
         * @brief Precompute reader-dependent full-scale value.
         */
        void precompute_() noexcept
        {
            fs_ = detail::FullScale<Reader>::value;
            if (fs_ <= 0)
                fs_ = 1;
        }

        /**
         * @brief Update turn counter when the phase wraps across 0/1 with hysteresis.
         * @param phase01 Current normalized phase in [0,1].
         * @note The first sample only seeds wrap tracking; it never creates a fake turn.
         */
        void accumulate_wrap_(float phase01) noexcept
        {
            // Clamp hysteresis to [0, 0.45] to prevent overlap.
            const float h = (cfg_.wrap_hyst < 0.f) ? 0.f : (cfg_.wrap_hyst > 0.45f ? 0.45f : cfg_.wrap_hyst);

            if (!has_last_phase_)
            {
                // Startup might begin near either end. Seed first, then compare real movement.
                last_phase01_ = phase01;
                has_last_phase_ = true;
                return;
            }

            // Detect wrap based on "far end" regions:
            // - low region  : [0, h]
            // - high region : [1-h, 1]
            //
            // If we were in high and now in low => +1 turn (forward wrap).
            // If we were in low and now in high => -1 turn (reverse wrap).
            const bool was_high = last_phase01_ > (1.f - h);
            const bool was_low = last_phase01_ < h;
            const bool is_high = phase01 > (1.f - h);
            const bool is_low = phase01 < h;

            if (was_high && is_low)
                ++turns_;
            else if (was_low && is_high)
                --turns_;

            last_phase01_ = phase01;
        }

        Config cfg_{};
        State st_{};
        int fs_{POTIO_DEFAULT_FULLSCALE};

        TimeFn time_fn_{nullptr};
        detail::DtState dt_{};

        float last_phase01_{0.f};
        int32_t turns_{0};
        bool has_sample_{false};
        bool has_last_phase_{false};
    };

} ///< namespace PotIO
