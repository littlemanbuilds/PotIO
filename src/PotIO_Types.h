/**
 * MIT License
 *
 * @brief Core public types, status contracts, and utilities for PotIO.
 *
 * @file PotIO_Types.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <math.h>

#include "PotIO_Compatibility.h"

namespace PotIO
{
    /** @brief Time source function type returning milliseconds. */
    using TimeFn = uint32_t (*)();

    /// @brief Small epsilon used for normalized float comparisons.
    static constexpr float kEps = 1e-7f;

    static constexpr float kPi = 3.14159265358979323846f;    ///< π constant.
    static constexpr float kRad2Deg = 57.29577951308232f;    ///< Radians-to-degrees scale.
    static constexpr float kDeg2Rad = 0.017453292519943295f; ///< Degrees-to-radians scale.

    /** @brief Convert radians to degrees. */
    inline float rad2deg(float r) noexcept { return r * kRad2Deg; }

    /** @brief Convert degrees to radians. */
    inline float deg2rad(float d) noexcept { return d * kDeg2Rad; }

    /**
     * @brief Primary reason the latest PotIO update did not produce fresh valid data.
     */
    enum class ReadError : uint8_t
    {
        None = 0,             ///< The latest update completed successfully.
        NoSample,             ///< No successful sample has been acquired yet.
        ReaderFailure,        ///< Reader explicitly reported failure or returned a negative value.
        OutOfRange,           ///< Reader returned a value outside the declared ADC range.
        CalibrationInvalid,   ///< Strict calibration policy rejected the configured calibration.
        InvalidTiming,        ///< dt was negative, NaN, infinity, or otherwise unusable.
        TimingGap,            ///< dt exceeded the configured maximum processing gap.
        InvalidConfiguration, ///< A built-in policy or device setting is invalid.
        Discontinuity         ///< ContinuousPot movement could not be tracked unambiguously.
    };

    /**
     * @brief Non-fatal quality flags attached to a successful sample.
     */
    enum QualityFlag : uint16_t
    {
        QualityNone = 0u,                    ///< No quality warning is active.
        QualityCalibrationFallback = 1u << 0 ///< Invalid calibration was ignored in permissive mode.
    };

    /**
     * @brief Reader result used when acquisition can explicitly fail.
     *
     * @details
     * Existing readers that return `int` remain supported. PotIO interprets a
     * negative integer as `ReaderFailure`. Custom readers can return RawSample
     * directly to distinguish an explicit failure from a real zero reading.
     */
    struct RawSample
    {
        int raw{0};                           ///< Raw ADC value when valid.
        bool valid{false};                    ///< True when `raw` is a real sample.
        ReadError error{ReadError::NoSample}; ///< Failure reason when `valid == false`.

        /** @brief Create a valid raw sample. */
        static RawSample success(int value) noexcept
        {
            RawSample s;
            s.raw = value;
            s.valid = true;
            s.error = ReadError::None;
            return s;
        }

        /** @brief Create a failed raw sample. */
        static RawSample failure(ReadError reason = ReadError::ReaderFailure) noexcept
        {
            RawSample s;
            s.raw = 0;
            s.valid = false;
            s.error = (reason == ReadError::None) ? ReadError::ReaderFailure : reason;
            return s;
        }
    };

    /**
     * @brief Status shared by PotIO device state frames.
     *
     * @details
     * Output values are retained across invalid updates. Always check `valid`
     * before treating the current frame as a fresh control input.
     * `sample_ms` and `sequence` refer only to successful acquisitions.
     */
    struct SampleStatus
    {
        bool valid{false};                    ///< True when the latest update produced a new valid sample.
        bool has_value{false};                ///< True after at least one valid sample has been retained.
        bool calibration_valid{false};        ///< True when configured calibration is valid for the reader range.
        ReadError error{ReadError::NoSample}; ///< Status of the latest update attempt.
        uint16_t quality{QualityNone};        ///< Bitwise OR of QualityFlag values.
        uint32_t sample_ms{0};                ///< Timestamp of the most recent successful sample.
        uint32_t sequence{0};                 ///< Monotonic count of successful samples; wraps naturally.
    };

    /**
     * @brief Potentiometer calibration (min/center/max).
     */
    struct PotCalib
    {
        uint16_t min{0};                                                     ///< Raw minimum value.
        uint16_t center{static_cast<uint16_t>(POTIO_DEFAULT_FULLSCALE / 2)}; ///< Raw physical center.
        uint16_t max{static_cast<uint16_t>(POTIO_DEFAULT_FULLSCALE)};        ///< Raw maximum value.

        PotCalib() = default;

        /** @brief Construct calibration from observed endpoints and center. */
        PotCalib(uint16_t min_, uint16_t center_, uint16_t max_) noexcept
            : min{min_}, center{center_}, max{max_} {}

        /** @brief True when the endpoint range is ordered. */
        POTIO_NODISCARD bool valid() const noexcept { return max > min; }

        /** @brief True when min, center, and max are strictly ordered. */
        POTIO_NODISCARD bool valid_centered() const noexcept
        {
            return (max > center) && (center > min);
        }

        /**
         * @brief Validate centered calibration against a reader full-scale.
         * @param fullscale Maximum valid raw value from the reader.
         * @param min_span Minimum raw units required on each side of center.
         * @return true when the calibration is ordered, in range, and has useful span.
         */
        POTIO_NODISCARD bool valid_centered_for(int fullscale, uint16_t min_span = 1u) const noexcept
        {
            if (fullscale <= 0 || !valid_centered())
                return false;
            if (static_cast<int>(max) > fullscale)
                return false;

            const uint32_t low_span = static_cast<uint32_t>(center) - static_cast<uint32_t>(min);
            const uint32_t high_span = static_cast<uint32_t>(max) - static_cast<uint32_t>(center);
            return low_span >= static_cast<uint32_t>(min_span) &&
                   high_span >= static_cast<uint32_t>(min_span);
        }
    };

    /**
     * @brief Behavior when configured calibration is unusable.
     */
    enum class CalibrationPolicy : uint8_t
    {
        PermissiveDefault, ///< Fall back to declared reader full-scale and set a quality flag.
        RequireValid       ///< Reject the update until a valid centered calibration is configured.
    };

    /**
     * @brief Processing-history behavior after an invalid sample.
     */
    enum class InvalidSamplePolicy : uint8_t
    {
        HoldState,      ///< Retain filter/rate history and output; default and least surprising.
        ResetProcessing ///< Retain public output but reseed filter/rate history on recovery.
    };

    /// @brief Deadzone strategies for joystick axes.
    enum class Deadzone : uint8_t
    {
        None,        ///< No deadzone.
        Axial,       ///< Zero each axis independently inside the threshold.
        AxialScaled, ///< Zero each axis then remap the remaining travel continuously to full scale.
        Radial,      ///< Circular deadzone without rescaling outside the zone.
        RadialScaled ///< Circular deadzone with continuous magnitude rescaling.
    };

    /**
     * @brief Geometry applied after joystick deadzone processing.
     */
    enum class JoystickGeometry : uint8_t
    {
        Square,         ///< Preserve independent X/Y authority; diagonal magnitude may exceed one internally.
        MagnitudeClamp, ///< Preserve angle but clamp vector magnitude to one.
        SquareToCircle  ///< Map the square axis domain smoothly into a unit circle.
    };

    /** @brief Clamp a value to [0,1]. */
    inline float clamp01(float v) noexcept { return v < 0.f ? 0.f : (v > 1.f ? 1.f : v); }

    /** @brief Clamp a value to [-1,1]. */
    inline float clamp11(float v) noexcept { return v < -1.f ? -1.f : (v > 1.f ? 1.f : v); }

} ///< namespace PotIO
