/**
 * MIT License
 *
 * @file 05_ESP32ADC.ino
 * @brief ESP32-only ADC attenuation and millivolt-reader example.
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 *
 * This is intentionally more technical than the earlier examples. ESP32 ADCs
 * have attenuation and millivolt helpers that are not part of generic Arduino.
 * Keep those details here so the beginner examples stay clean.
 */

#include <PotIO.h>

#include <Arduino.h>

#if !defined(ARDUINO_ARCH_ESP32)
#error "This example is ESP32-only."
#endif

// GPIO4 is used here because it is a simple ADC-capable example pin on
// ESP32-S3 boards. If your board routes ADC differently, change this pin.
static constexpr uint8_t kPotPin = 4;

// 11 dB is a common Arduino-ESP32 attenuation choice for reading a wider range.
// Always check your board and sensor wiring: ADC pins are not 5 V tolerant.
static constexpr int kAttenDb = 11;
static constexpr int kFullScaleMv = 3300;

// Replace these with values measured on your own board using this example.
static const PotIO::PotCalib kObservedMv{80, 1640, 3190};

auto potRaw = PotIO::makePotAnalog(kPotPin, kAttenDb);
auto potMv = PotIO::makePotMilliVolts<kFullScaleMv>(kPotPin, kObservedMv, kAttenDb);

void setup()
{
    Serial.begin(115200);
    delay(100);

    PotIO::setEsp32PinAttenuation(kPotPin, kAttenDb);

    Serial.println();
    Serial.println("PotIO 05 - ESP32 ADC raw-count vs millivolt reader");
    Serial.println("Use this when you want to understand what the ESP32 ADC is really seeing.");
}

void loop()
{
    potRaw.update();
    potMv.update();

    const auto raw = potRaw.state();
    const auto mv = potMv.state();
    if (!raw.status.valid || !mv.status.valid)
    {
        Serial.print("sample invalid  raw_valid=");
        Serial.print(raw.status.valid ? 1 : 0);
        Serial.print("  mv_valid=");
        Serial.println(mv.status.valid ? 1 : 0);
        delay(20);
        return;
    }

    Serial.print("raw01=");
    Serial.print(raw.raw01, 3);
    Serial.print("  raw_centered=");
    Serial.print(raw.centered, 3);
    Serial.print("  mv01=");
    Serial.print(mv.raw01, 3);
    Serial.print("  mv_centered=");
    Serial.println(mv.centered, 3);

    delay(20);
}
