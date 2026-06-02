/**
 * MIT License
 *
 * @brief Calibrated linear potentiometer with filtering, shaping, and slew limiting.
 *
 * @file PotIO_LinearPot.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>

#include <PotIO_Detail.h>
#include <PotIO_Filters.h>
#include <PotIO_RateLimit.h>
#include <PotIO_Shaping.h>

namespace PotIO
{
    /**
     * @brief Linear potentiometer device: raw → calibrated → centered (-1..+1).
     *
     * @details
     * This class is intentionally single-threaded: call update() from one context.
     * If you need cross-task/cross-core consumers, publish a POD frame using SnapshotBus.
     *
     * @tparam Reader Source functor (e.g., ArduinoAnalogRead). May define `static constexpr int kFullScale`.
     * @tparam Filter Sample filter (e.g., EMAFilter).
     * @tparam Rate Rate limiter (e.g., NoRateLimit or SlewRate).
     * @tparam Shaper Nonlinear shaper (e.g., ShapeIdentity).
     */
    template <typename Reader,
              typename Filter = EMAFilter,
              typename Rate = NoRateLimit,
              typename Shaper = ShapeIdentity>
    class LinearPot
    {
    public:
        /**
         * @brief Configuration bundle for LinearPot.
         */
        struct Config
        {
            Reader reader{};  ///< ADC reader functor.
            PotCalib calib{}; ///< Calibration (min/center/max). If invalid, raw normalization is used.
            Filter filter{};  ///< Filter instance.
            Rate rate{};      ///< Rate limiting instance.
            Shaper shape{};   ///< Response shaper.
        };

        /**
         * @brief Public state produced by update().
         */
        struct State
        {
            float raw01{0.f};    ///< Raw reading normalized to [0,1].
            float calib01{0.f};  ///< Calibrated value normalized to [0,1].
            float centered{0.f}; ///< Centered value in [-1,1] after shape/filter/rate.
            uint32_t t_ms{0};    ///< Timestamp in milliseconds of last update.
        };

        /**
         * @brief POD snapshot suitable for publishing via SnapshotBus.
         */
        using Frame = State;

        LinearPot() = default;

        /**
         * @brief Construct with configuration.
         * @param c Configuration bundle.
         */
        explicit LinearPot(const Config &c) : cfg_(c) { precompute_(); }

        // ---- Time-source ergonomics ---- //

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
         * @brief Update using an external timestamp; dt is computed internally (wrap-safe).
         * @param now_ms Current timestamp in milliseconds.
         */
        void update(uint32_t now_ms) noexcept
        {
            const float dt_s = dt_.step(now_ms);
            update(now_ms, dt_s);
        }

        /**
         * @brief Update the device state from the current ADC reading.
         *
         * @param now_ms Current time in milliseconds.
         * @param dt_s Time step in seconds (for filters/rate limits).
         */
        void update(uint32_t now_ms, float dt_s) noexcept
        {
            const int fullscale = adc_full_scale_;
            const int raw = detail::clamp_raw(cfg_.reader(), fullscale);

            const float raw01 = adc_full_scale_inv_ * static_cast<float>(raw);
            const float v01 = detail::map_raw_to_01(raw, cfg_.calib, fullscale);

            float ctr = detail::to_centered(v01);
            ctr = cfg_.shape(ctr);
            if (has_sample_)
            {
                ctr = cfg_.filter(st_.centered, ctr);
                ctr = cfg_.rate(st_.centered, ctr, dt_s);
            }
            else
            {
                // First sample seeds the output, so filters/slew limits do not ramp up from zero.
                has_sample_ = true;
            }

            st_.raw01 = raw01;
            st_.calib01 = v01;
            st_.centered = clamp11(ctr);
            st_.t_ms = now_ms;
        }

        /**
         * @brief Get a const reference to the latest state.
         * @return Latest state storage owned by this object.
         */
        POTIO_NODISCARD const State &state() const noexcept { return st_; }

        /**
         * @brief Get a POD frame (copy) for publishing via SnapshotBus.
         * @return Copy of the latest state.
         */
        POTIO_NODISCARD Frame frame() const noexcept { return st_; }

        /**
         * @brief Get the calibrated value in [0,1].
         * @return Latest calibrated normalized value.
         */
        POTIO_NODISCARD float calib01() const noexcept { return st_.calib01; }

        /**
         * @brief Get the centered value in [-1,1].
         * @return Latest shaped, filtered, and rate-limited centered value.
         */
        POTIO_NODISCARD float centered() const noexcept { return st_.centered; }

        /**
         * @brief Set a new configuration and recompute reader-dependent fields.
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
         * @brief Get the effective ADC full-scale used for normalization (Reader::kFullScale or default).
         * @return Full-scale value in raw reader units.
         */
        POTIO_NODISCARD int fullScale() const noexcept { return adc_full_scale_; }

    private:
        /**
         * @brief Precompute reader-dependent scaling constants.
         */
        void precompute_() noexcept
        {
            adc_full_scale_ = detail::FullScale<Reader>::value;
            if (adc_full_scale_ <= 0)
                adc_full_scale_ = 1;
            adc_full_scale_inv_ = 1.0f / static_cast<float>(adc_full_scale_);
        }

        Config cfg_{};                 ///< Active configuration.
        State st_{};                   ///< Public state storage.
        int adc_full_scale_{POTIO_DEFAULT_FULLSCALE}; ///< Reader full scale (raw units).
        float adc_full_scale_inv_{1.f / static_cast<float>(POTIO_DEFAULT_FULLSCALE)}; ///< Cached 1/fullscale.
        TimeFn time_fn_{nullptr};      ///< Optional injected time source (ms).
        detail::DtState dt_{};         ///< dt computation state (wrap-safe, first-call guarded).
        bool has_sample_{false};       ///< True once the output has been seeded from a real sample.
    };

} ///< namespace PotIO
