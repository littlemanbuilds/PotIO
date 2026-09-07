/**
 * MIT License
 *
 * @file 07_PotSteering.ino
 * @brief Use a potentiometer as steering-style normalized input.
 *
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 *
 * This example is about input interpretation, not motor control. PotIO reads a
 * steering-style pot and gives a clean centered value:
 *
 *   -1.0 = left side of calibrated travel
 *    0.0 = calibrated center
 *   +1.0 = right side of calibrated travel
 *
 * A separate control layer, such as SteerCore or your vehicle project, decides
 * what to do with that input.
 */

#include <PotIO.h>

#include <Arduino.h>

using namespace PotIO;

#if defined(ARDUINO_ARCH_ESP32)
// GPIO4 is used here because it is a simple ADC-capable example pin on
// ESP32-S3 boards. If your board routes ADC differently, change this pin.
static constexpr int kPotPin = 4;
#else
static constexpr int kPotPin = A0;
#endif

#if defined(ARDUINO_ARCH_ESP32)
static const PotCalib kExampleSteeringCalib{200, 2048, 3900};
#else
static const PotCalib kExampleSteeringCalib{50, 512, 975};
#endif

using SteeringPot = LinearPot<ArduinoAnalogRead,
                                     EMAFilter,
                                     SlewRate,
                                     ShapeIdentity>;

static SteeringPot makeSteeringPot()
{
    SteeringPot::Config cfg;
    cfg.reader = ArduinoAnalogRead(kPotPin);

    // Replace this with values from 02_PotCalibration for your own hardware.
    // The example constants are board-family friendly guesses, not exact
    // steering values.
    cfg.calib = kExampleSteeringCalib;

    // A little smoothing removes ADC twitch without hiding the live input.
    cfg.filter.alpha = 0.12f;

    // Slew limiting prevents sudden jumps in the interpreted input value.
    // It does not drive a motor and it is not a safety system.
    cfg.rate.units_per_s = 5.0f;

    return SteeringPot(cfg);
}

static int toDisplayPercent(float centered)
{
    const float t = clamp01((centered + 1.0f) * 0.5f);
    return static_cast<int>((t * 100.0f) + 0.5f);
}

static SteeringPot steering = makeSteeringPot();
static JitterStats jitter{};

void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("PotIO 07 - Steering-style input");
    Serial.println("PotIO measures the input. It does not drive the steering motor.");
}

void loop()
{
    steering.update();
    const auto s = steering.state();
    if (!s.status.valid)
    {
        Serial.println("sample invalid - steering input unavailable");
        delay(20);
        return;
    }

    // JitterStats watches the raw ADC movement and helps estimate a sensible
    // deadzone. It is a measuring aid, not an automatic tuning system.
    const uint16_t rawCounts = static_cast<uint16_t>((s.raw01 * static_cast<float>(steering.fullScale())) + 0.5f);
    jitter.observe(rawCounts);

    const float suggestedDeadzone =
        suggest_deadzone_from_jitter(jitter.peak_to_peak(),
                                            static_cast<uint16_t>(steering.fullScale()),
                                            5);

    Serial.print("centered=");
    Serial.print(s.centered, 3);
    Serial.print("  display_percent=");
    Serial.print(toDisplayPercent(s.centered));
    Serial.print("  suggested_deadzone=");
    Serial.println(suggestedDeadzone, 3);

    delay(20);
}
