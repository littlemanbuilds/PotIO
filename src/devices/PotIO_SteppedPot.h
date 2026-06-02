/**
 * MIT License
 *
 * @brief Discrete-step potentiometer (maps analog input to N stable positions).
 *
 * @file PotIO_SteppedPot.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include <PotIO_Detail.h>
#include <PotIO_Filters.h>
#include <PotIO_Arduino.h>

namespace PotIO
{
    /**
     * @brief Map an analog potentiometer to N discrete steps with hysteresis.
     *
     * @details
     * Useful for "gear selectors", mode knobs, or any input where you want stable
     * discrete positions from an analog potentiometer.
     *
     * @tparam Steps Number of discrete steps (>=2).
     * @tparam Reader ADC reader functor.
     * @tparam Filter Optional filter applied to normalized [0..1] before quantization.
     */
    template <size_t Steps,
              typename Reader = ArduinoAnalogRead,
              typename Filter = NoFilter>
    class SteppedPot
    {
        static_assert(Steps >= 2, "SteppedPot<Steps>: Steps must be >= 2.");
        static_assert(Steps <= 255, "SteppedPot<Steps>: Steps must fit in uint8_t.");

    public:
        /**
         * @brief Configuration bundle for SteppedPot.
         */
        struct Config
        {
            Reader reader{};            ///< ADC reader.
            PotCalib calib{};           ///< Calibration; if invalid -> raw normalization.
            Filter filter{};            ///< Optional filter on v01.
            float hysteresis{0.04f};    ///< Hysteresis as fraction of a step width (0..0.49).
        };

        /**
         * @brief Public state produced by update().
         */
        struct State
        {
            uint16_t raw{0};     ///< Raw ADC sample.
            float v01{0.f};      ///< Filtered normalized reading in [0..1].
            uint8_t step{0};     ///< Current step (0..Steps-1).
            bool changed{false}; ///< True if step changed on the last update.
            uint32_t t_ms{0};    ///< Timestamp in ms.
        };

        /**
         * @brief POD snapshot suitable for publishing via SnapshotBus.
         */
        using Frame = State;

        SteppedPot() = default;

        /**
         * @brief Construct with configuration.
         * @param c Configuration bundle.
         */
        explicit SteppedPot(const Config &c) : cfg_(c) { precompute_(); }

        /**
         * @brief Inject a custom time source (milliseconds). Pass nullptr to revert to default (Arduino::millis()).
         * @param fn Optional function returning current milliseconds.
         */
        void setTimeSource(TimeFn fn) noexcept { time_fn_ = fn; }

        /**
         * @brief Update using internal time source.
         */
        void update() noexcept { update(detail::time_now_ms(time_fn_)); }

        /**
         * @brief Update using external timestamp; dt is computed internally.
         * @param now_ms Current timestamp in milliseconds.
         */
        void update(uint32_t now_ms) noexcept { update(now_ms, dt_.step(now_ms)); }

        /**
         * @brief Update using explicit timestamp and time step.
         * @param now_ms Current timestamp in milliseconds.
         * @param dt_s Elapsed time in seconds; accepted for API consistency.
         *
         * @note The first update seeds the current step and reports `changed=false`.
         */
        void update(uint32_t now_ms, float dt_s) noexcept
        {
            (void)dt_s; // The default Filter uses prev/sample, so it does not need dt_s.

            const int raw_i = detail::clamp_raw(cfg_.reader(), fs_);
            const float v01_in = detail::map_raw_to_01(raw_i, cfg_.calib, fs_);
            float v01 = v01_in;
            if (has_sample_)
                v01 = cfg_.filter(st_.v01, v01_in);
            else
                has_sample_ = true;
            v01 = clamp01(v01);

            const uint8_t prev_step = st_.step;
            const uint8_t next_step = has_step_ ? quantize_(v01, prev_step) : quantize_first_(v01);
            const bool changed = has_step_ && (next_step != prev_step);
            has_step_ = true;

            st_.raw = static_cast<uint16_t>(raw_i);
            st_.v01 = v01;
            st_.step = next_step;
            st_.changed = changed;
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
         * @brief Replace configuration.
         * @param c New configuration bundle.
         * @param reset_state If true, clear filter history, timing, and current step.
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
            has_step_ = false;
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
         * @brief Get normalized width of one step.
         * @return Step width in [0,1] units.
         */
        static inline float step_width_() noexcept { return 1.0f / static_cast<float>(Steps); }

        /**
         * @brief Quantize a value without hysteresis for the first seeded step.
         * @param v01 Normalized value in [0,1].
         * @return Step index in [0, Steps-1].
         */
        uint8_t quantize_first_(float v01) const noexcept
        {
            const float w = step_width_();
            const int idx = static_cast<int>(floor(v01 / w));
            return static_cast<uint8_t>((idx < 0) ? 0 : (idx >= static_cast<int>(Steps) ? static_cast<int>(Steps) - 1 : idx));
        }

        /**
         * @brief Quantize a value while holding the current step inside a hysteresis band.
         * @param v01 Normalized value in [0,1].
         * @param current Current step index.
         * @return Next stable step index.
         */
        uint8_t quantize_(float v01, uint8_t current) const noexcept
        {
            // Nominal step index from value:
            const float w = step_width_();
            const uint8_t q = quantize_first_(v01);

            // Hysteresis: only transition out of the current step once we move past a band.
            const float h = (cfg_.hysteresis < 0.f) ? 0.f : (cfg_.hysteresis > 0.49f ? 0.49f : cfg_.hysteresis);
            const float band = h * w;

            const float lo = static_cast<float>(current) * w - band;
            const float hi = static_cast<float>(current + 1) * w + band;

            // If still in the widened band of the current step, hold it.
            if (v01 >= lo && v01 <= hi)
                return current;

            return q;
        }

        Config cfg_{};
        State st_{};

        int fs_{POTIO_DEFAULT_FULLSCALE};
        TimeFn time_fn_{nullptr};
        detail::DtState dt_{};
        bool has_sample_{false};
        bool has_step_{false};
    };

} ///< namespace PotIO
