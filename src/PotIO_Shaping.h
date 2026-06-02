/**
 * MIT License
 *
 * @brief Nonlinear shaping (response curves) for joystick/pot signals.
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
    /**
     * @brief Identity (no shaping).
     */
    struct ShapeIdentity
    {
        /**
         * @brief Apply shaping to input.
         * @param x Input in [-1,1].
         * @return float Output in [-1,1] (unchanged).
         */
        float operator()(float x) const noexcept { return x; }
    };

    /**
     * @brief Cubic/expo blend: x * (a + (1-a) * x^2), a ∈ [0,1].
     */
    struct ShapeCubicExpo
    {
        float a{0.7f}; ///< Blend factor (0..1). 0=cubic, 1=linear.

        /**
         * @brief Apply cubic/expo shaping.
         * @param x Input in [-1,1].
         * @return float Output in [-1,1].
         */
        float operator()(float x) const noexcept
        {
            const float aa = (a < 0.f) ? 0.f : (a > 1.f ? 1.f : a);
            return x * (aa + (1.f - aa) * x * x);
        }
    };

    /**
     * @brief Soft dead zone using normalized tanh.
     */
    struct ShapeSoftZone
    {
        float k{3.0f}; ///< Gain; >= 1. Larger = snappier around zero.

        /**
         * @brief Apply soft-zone shaping.
         * @param x Input in [-1,1].
         * @return float Output in [-1,1].
         */
        float operator()(float x) const noexcept
        {
            const float kk = (k < 1.f) ? 1.f : k;
            return tanh(kk * x) / tanh(kk);
        }
    };

} ///< namespace PotIO
