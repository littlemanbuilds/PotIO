/**
 * MIT License
 *
 * @file 04_Joystick2DKY023.ino
 * @brief Read a KY-023 style analog joystick with a radial scaled deadzone.
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 *
 * A joystick is just two analog inputs that share one idea of "center". PotIO
 * reads both axes, applies calibration, removes the small wobbly zone around
 * center, and reports x/y values that are easier for a control layer to use.
 */

#include <PotIO.h>

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
// GPIO4/GPIO5 are used here because they are simple ADC-capable example pins
// on ESP32-S3 boards. If your board routes ADC differently, change these pins.
static constexpr uint8_t kPinX = 4;
static constexpr uint8_t kPinY = 5;
#elif defined(ARDUINO_ARCH_ESP8266)
static constexpr uint8_t kPinX = A0;
static constexpr uint8_t kPinY = A0; // Compile fallback only; most ESP8266 boards expose one ADC pin.
#else
static constexpr uint8_t kPinX = A0;
static constexpr uint8_t kPinY = A1;
#endif

// RadialScaled means: ignore the small center wobble, then scale the remaining
// movement so the joystick can still reach full output near the edge.
auto joystick = PotIO::makeJoystickKY023<>(kPinX, kPinY, PotIO::Deadzone::RadialScaled, 0.12f);

void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("PotIO 04 - Joystick2D KY-023");
    Serial.println("Move the joystick and watch x/y/magnitude/angle.");
#if defined(ARDUINO_ARCH_ESP8266)
    Serial.println("ESP8266 note: most boards need an external ADC or another board for true two-axis joystick input.");
#endif
}

void loop()
{
    joystick.update();

    const auto s = joystick.state();
    if (!s.status.valid)
    {
        Serial.println("joystick sample invalid - holding the last good output");
        delay(20);
        return;
    }

    Serial.print("x=");
    Serial.print(s.x, 3);
    Serial.print("  y=");
    Serial.print(s.y, 3);
    Serial.print("  mag=");
    Serial.print(s.mag, 3);
    Serial.print("  angle_deg=");
    if (joystick.angleValid())
        Serial.println(joystick.angleDeg(), 1);
    else
        Serial.println("n/a");

    delay(20);
}
