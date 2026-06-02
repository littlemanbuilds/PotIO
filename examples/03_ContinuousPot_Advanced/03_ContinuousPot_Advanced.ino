/**
 * @file 03_ContinuousPot_Advanced.ino
 * @brief Use a cyclic analog knob with phase, wrap tracking, and unwrapped angle.
 *
 * ContinuousPot is for controls where crossing the 0/1 boundary should be
 * treated as moving into the next turn. It can feel like a jog wheel or rotary
 * control, but the hardware still matters: a normal potentiometer is not truly
 * endless unless the physical signal is cyclic.
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

// Sticky is a friendly preset: a little smoothing and a soft response curve.
// Other presets are shown in setup() so the beginner can discover them without
// turning the first line into template soup.
auto knob = PotIO::makeKnobSticky(kPotPin);

static int32_t lastTurns = 0;

void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("PotIO 03 - ContinuousPot advanced");
    Serial.println("Watch phase, turns, and unwrapped angle.");
    Serial.println();
    Serial.println("Other useful presets:");
    Serial.println("  PotIO::makeKnobSmooth(pin)");
    Serial.println("  PotIO::makeKnobJog(pin)");
}

void loop()
{
    knob.update();
    const auto s = knob.state();

    const int32_t turns = static_cast<int32_t>(knob.turns_raw());
    if (turns != lastTurns)
    {
        Serial.print("wrap detected: ");
        Serial.println((turns > lastTurns) ? "+1 turn" : "-1 turn");
        lastTurns = turns;
    }

    Serial.print("raw=");
    Serial.print(s.raw);
    Serial.print("  phase01=");
    Serial.print(s.phase01, 3);
    Serial.print("  centered=");
    Serial.print(s.centered, 3);
    Serial.print("  turns=");
    Serial.print(knob.turns_raw(), 0);
    Serial.print("  angle_deg=");
    Serial.println(knob.angleUnwrappedDeg(), 1);

    delay(20);
}
