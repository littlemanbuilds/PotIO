/**
 * MIT License
 *
 * @file 01_LinearPotBasic.ino
 * @brief Read one potentiometer and print beginner-friendly normalized values.
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 *
 * This is the "hello world" example for PotIO. The important idea is that the
 * library turns a noisy board-specific ADC count into values that are easier to
 * reason about:
 *
 *   raw01    = 0.0 .. 1.0
 *   centered = -1.0 .. +1.0
 *
 * PotIO does not decide what the value means. Your project decides whether this
 * is a throttle, trim knob, steering input, menu control, or something else.
 */

#include <PotIO.h>
#include <Arduino.h>

// Little Man Builds projects currently develop on the ESP32-S3 DevKitC-1.
// GPIO4 is used here because it is a simple ADC-capable example pin on
// ESP32-S3 boards. If your board routes ADC differently, change this pin.
#if defined(ARDUINO_ARCH_ESP32)
static constexpr uint8_t kPotPin = 4;
#else
static constexpr uint8_t kPotPin = A0;
#endif

// The factory creates a LinearPot using Arduino analogRead().
// By default PotIO assumes ESP32-style 12-bit ADC on ESP32 and 10-bit ADC on
// classic Arduino boards. If your board uses a different ADC resolution, define
// POTIO_ARDUINO_ADC_FULLSCALE before including PotIO.h or use a custom reader.
auto pot = PotIO::makePotAnalog(kPotPin);

void setup()
{
    Serial.begin(115200);
    delay(100);

    Serial.println();
    Serial.println("PotIO 01 - Basic linear potentiometer");
    Serial.println("Move the pot and watch raw01 and centered change.");
}

void loop()
{
    // update() reads the ADC once, maps it, and stores a complete latest state.
    // There are no hidden delays inside PotIO.
    pot.update();

    const auto s = pot.state();
    if (!s.status.valid)
    {
        Serial.println("sample invalid - holding the last good output");
        delay(20);
        return;
    }

    Serial.print("raw01=");
    Serial.print(s.raw01, 3);
    Serial.print("  centered=");
    Serial.println(s.centered, 3);

    delay(20);
}
