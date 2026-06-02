/**
 * MIT License
 *
 * @brief Compatibility layer and configuration defaults for PotIO.
 *
 * @file PotIO_Compatibility.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

// ---- Platform detection ---- //

#if defined(ARDUINO)
#include <Arduino.h>
#endif

// ---- Version macro ---- //

#define POTIO_VERSION "1.0.0"

// ---- Tunables ---- //
//
// Users may define these macros before including <PotIO.h> to override defaults.
//

/**
 * @brief Default ADC full-scale used when a Reader does not provide T::kFullScale and no calibration is set.
 *
 * @details
 * PotIO does not call analogReadResolution() or similar APIs (as they are platform-specific).
 * If your platform/resolution differs from these defaults, either:
 *  - define POTIO_ARDUINO_ADC_FULLSCALE (recommended for Arduino builds), or
 *  - provide a Reader type that defines `static constexpr int kFullScale = ...;`.
 */
#ifndef POTIO_ARDUINO_ADC_FULLSCALE
#if defined(ARDUINO_ARCH_ESP32) || defined(ESP_PLATFORM) || defined(ESP32)
#define POTIO_ARDUINO_ADC_FULLSCALE 4095
#else
#define POTIO_ARDUINO_ADC_FULLSCALE 1023
#endif
#endif

#ifndef POTIO_DEFAULT_FULLSCALE
#if defined(ARDUINO)
#define POTIO_DEFAULT_FULLSCALE POTIO_ARDUINO_ADC_FULLSCALE
#else
// Non-Arduino builds typically inject a Reader with an explicit kFullScale.
#define POTIO_DEFAULT_FULLSCALE 4095
#endif
#endif

// ---- Assertions ---- //
//
// A library should never assume Serial exists or is initialized.
// By default we use the C assert header because older Arduino cores may not ship <cassert>.
//

#ifndef POTIO_ASSERT
#include <assert.h>
#define POTIO_ASSERT(x) assert(x)
#endif

#ifndef POTIO_UNUSED
#define POTIO_UNUSED(x) (void)(x)
#endif

#ifndef POTIO_NODISCARD
#if __cplusplus >= 201703L
#define POTIO_NODISCARD [[nodiscard]]
#else
#define POTIO_NODISCARD
#endif
#endif
