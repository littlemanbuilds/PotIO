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

        /**
         * @brief Pass through an input without shaping.
         *
         * @param x Input value.
         * @return x unchanged.
         */
        float operator()(float x) const noexcept { return x; }
    };

    /** @brief Cubic/expo blend: x * (a + (1-a) * x²). */
    struct ShapeCubicExpo
    {
        float a{0.7f}; ///< Blend factor in [0,1]. 0=cubic, 1=linear.

        /**
         * @brief Validate the blend factor.
         *
         * @return True for finite a in [0,1].
         */
        bool valid() const noexcept
        {
            return isfinite(a) != 0 && a >= 0.0f && a <= 1.0f;
        }

        /**
         * @brief Blend cubic and linear response without clamping the input.
         *
         * @param x Centered input, normally in [-1,1].
         * @return Shaped value, or x when a is invalid.
         */
        float operator()(float x) const noexcept
        {
            if (!valid())
                return x;
            return x * (a + (1.f - a) * x * x);
        }
    };

    /** @brief Normalized tanh response with increased center sensitivity and flattening near the endpoints. */
    struct ShapeSoftZone
    {
        float k{3.0f}; ///< Gain; must be finite and >= 1.

        /**
         * @brief Validate the configured gain.
         *
         * @return True for finite k at least one.
         */
        bool valid() const noexcept
        {
            return isfinite(k) != 0 && k >= 1.0f;
        }

        /**
         * @brief Apply normalized tanh response without clamping the input.
         *
         * @param x Centered input, normally in [-1,1].
         * @return Shaped value, or x when k is invalid.
         */
        float operator()(float x) const noexcept
        {
            if (!valid())
                return x;
            return tanh(k * x) / tanh(k);
        }
    };

} ///< namespace PotIO
