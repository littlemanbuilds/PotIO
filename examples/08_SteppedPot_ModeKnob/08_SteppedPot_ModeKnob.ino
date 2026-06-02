/**
 * @file 08_SteppedPot_ModeKnob.ino
 * @brief Convert one potentiometer into a stable 5-position mode knob.
 *
 * SteppedPot is useful when a cheap analog pot should behave like a selector:
 * Eco / Normal / Sport, menu pages, brightness presets, tuning profiles, etc.
 *
 * Hysteresis keeps the output from flickering when the ADC value sits near a
 * boundary between two steps.
 */

#include <Arduino.h>
#include <PotIO.h>

#if defined(ARDUINO_ARCH_ESP32)
// GPIO4 is used here because it is a simple ADC-capable example pin on
// ESP32-S3 boards. If your board routes ADC differently, change this pin.
static constexpr uint8_t kPotPin = 4;
#else
static constexpr uint8_t kPotPin = A0;
#endif

// Five stable positions: 0, 1, 2, 3, 4.
auto modeKnob = PotIO::makeSteppedPot<5>(kPotPin);

static const char *modeName(uint8_t step)
{
    switch (step)
    {
    case 0:
        return "Eco";
    case 1:
        return "Normal";
    case 2:
        return "Sport";
    case 3:
        return "Setup";
    case 4:
        return "Debug";
    default:
        return "Unknown";
    }
}

void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("PotIO 08 - SteppedPot mode knob");
    Serial.println("Turn the pot until it crosses into a new stable step.");
}

void loop()
{
    modeKnob.update();

    const auto s = modeKnob.state();
    if (s.changed)
    {
        Serial.print("mode=");
        Serial.print(s.step);
        Serial.print(" (");
        Serial.print(modeName(s.step));
        Serial.print(")  v01=");
        Serial.println(s.v01, 3);
    }

    delay(20);
}
