/**
 * MIT License
 *
 * @brief Nonlinear response shaping for normalized PotIO signals.
 *
 * @file PotIO_Shaping.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <math.h>

namespace PotIO
{
    /** @brief Identity response curve. */
    struct ShapeIdentity
    {
        /**
 * @brief Validate configuration.
 *
 * @return Always true.
 */
        bool valid() const noexcept { return true; }

        /** @brief Apply identity shaping. */
        float operator()(float x) const noexcept { return x; }
    };

    /** @brief Cubic/expo blend: x * (a + (1-a) * x²). */
    struct ShapeCubicExpo
    {
        float a{0.7f}; ///< Blend factor in [0,1]. 0=cubic, 1=linear.

        /** @brief Validate the blend factor. */
        bool valid() const noexcept
        {
            return isfinite(a) != 0 && a >= 0.0f && a <= 1.0f;
        }

        /** @brief Apply cubic/expo shaping. */
        float operator()(float x) const noexcept
        {
            if (!valid())
                return x;
            return x * (a + (1.f - a) * x * x);
        }
    };

    /** @brief Normalized tanh response with a gentle center region. */
    struct ShapeSoftZone
    {
        float k{3.0f}; ///< Gain; must be finite and >= 1.

        /** @brief Validate the configured gain. */
        bool valid() const noexcept
        {
            return isfinite(k) != 0 && k >= 1.0f;
        }

        /** @brief Apply soft-zone shaping. */
        float operator()(float x) const noexcept
        {
            if (!valid())
                return x;
            return tanh(k * x) / tanh(k);
        }
    };

} ///< namespace PotIO
