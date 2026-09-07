/**
 * MIT License
 *
 * @file 06_PotShaping.ino
 * @brief Compare two response curves for analog controls.
 *
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 *
 * Shaping changes the feel of a control after it has been calibrated and
 * centered. Cubic expo reduces sensitivity near center for finer control; the
 * tanh curve increases center sensitivity and flattens toward the edges.
 *
 * This example uses two ADC pins so you can compare two pots side-by-side. If
 * you only have one pot, wire the same wiper to both example pins or change both
 * pin constants to the same ADC pin.
 */

#include <PotIO.h>

#include <Arduino.h>

using namespace PotIO;

#if defined(ARDUINO_ARCH_ESP32)
// GPIO4/GPIO5 are used here because they are simple ADC-capable example pins
// on ESP32-S3 boards. If your board routes ADC differently, change these pins.
static constexpr int kPinExpo = 4;
static constexpr int kPinSoft = 5;
#elif defined(ARDUINO_ARCH_ESP8266)
static constexpr int kPinExpo = A0;
static constexpr int kPinSoft = A0; // Compare two curves on the same ADC input.
#else
static constexpr int kPinExpo = A0;
static constexpr int kPinSoft = A1;
#endif

using ExpoPot = LinearPot<ArduinoAnalogRead,
                                 EMAFilter,
                                 SlewRate,
                                 ShapeCubicExpo>;

using SoftPot = LinearPot<ArduinoAnalogRead,
                                 EMAFilter,
                                 SlewRate,
                                 ShapeSoftZone>;

static ExpoPot makeExpoPot(uint8_t pin)
{
    ExpoPot::Config cfg;
    cfg.reader = ArduinoAnalogRead(pin);

    // Lower alpha smooths more. Higher alpha follows faster.
    cfg.filter.alpha = 0.12f;

    // Limit how quickly the output can move, in normalized units per second.
    cfg.rate.units_per_s = 3.0f;

    // Cubic-expo blends linear and cubic feel.
    cfg.shape.a = 0.6f;

    return ExpoPot(cfg);
}

static SoftPot makeSoftPot(uint8_t pin)
{
    SoftPot::Config cfg;
    cfg.reader = ArduinoAnalogRead(pin);
    cfg.filter.alpha = 0.12f;
    cfg.rate.units_per_s = 3.0f;

    // Soft-zone increases sensitivity near center and flattens toward the edges.
    cfg.shape.k = 4.0f;

    return SoftPot(cfg);
}

// The pots are built after their shaping parameters are set inside the factory
// functions above. This avoids the easy mistake of changing a policy object after
// it has already been copied into the LinearPot.
auto potExpo = makeExpoPot(kPinExpo);
auto potSoft = makeSoftPot(kPinSoft);

void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("PotIO 06 - Pot shaping");
    Serial.println("Compare cubic-expo and soft-zone response curves.");
#if defined(ARDUINO_ARCH_ESP8266)
    Serial.println("ESP8266 note: most boards have one ADC, so both curves read the same input.");
#endif
}

void loop()
{
    potExpo.update();
    potSoft.update();

    if (!potExpo.valid() || !potSoft.valid())
    {
        Serial.println("sample invalid - output unavailable; previous values retained");
        delay(20);
        return;
    }

    Serial.print("expo_centered=");
    Serial.print(potExpo.centered(), 3);
    Serial.print("  soft_centered=");
    Serial.println(potSoft.centered(), 3);

    delay(20);
}
