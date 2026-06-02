/**
 * MIT License
 *
 * @brief PotIO: potentiometer & joystick I/O with calibration, shaping, filtering, and factory helpers.
 *
 * @file PotIO.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 */

#pragma once

#include <PotIO_Compatibility.h>
#include <PotIO_Types.h>
#include <PotIO_Filters.h>
#include <PotIO_RateLimit.h>
#include <PotIO_Shaping.h>
#include <PotIO_JitterTools.h>

// Platform adapters (optional).
#include <PotIO_Arduino.h>

// Devices.
#include <devices/PotIO_LinearPot.h>
#include <devices/PotIO_ContinuousPot.h>
#include <devices/PotIO_Joystick2D.h>
#include <devices/PotIO_SteppedPot.h>

// Helpers & presets.
#include <devices/helpers/PotIO_ContinuousPotHelpers.h>
#include <devices/helpers/PotIO_AnalogStick.h>

// Factories.
#include <PotIO_Factory.h>
