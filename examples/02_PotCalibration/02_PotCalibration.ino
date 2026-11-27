/**
 * MIT License
 *
 * @file 02_PotCalibration.ino
 * @brief Measure real min, center, and max values for PotCalib.
 *
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-02
 * @copyright Copyright © 2026 Little Man Builds
 *
 * Real potentiometers rarely use the perfect full ADC range. The ends may stop
 * before 0 or before full scale, and the physical center may not be exactly the
 * mathematical center.
 *
 * This example keeps the flow deliberately simple:
 *   1. Move to the low end and press ENTER.
 *   2. Move to the physical center and press ENTER.
 *   3. Move to the high end and press ENTER.
 *   4. Copy the printed PotCalib into your real sketch.
 */

#include <PotIO.h>

#include <Arduino.h>

using namespace PotIO;

#if defined(ARDUINO_ARCH_ESP32)
// GPIO4 is used here because it is a simple ADC-capable example pin on
// ESP32-S3 boards. If your board routes ADC differently, change this pin.
const int kPotPin = 4;
#else
const int kPotPin = A0;
#endif

// Averaging a small number of samples makes the captured calibration point more
// repeatable without turning this into a complicated data-logging sketch.
const int kSamples = 64;
const int kSampleDelayMs = 3;

static void waitForEnter(const char *prompt)
{
    Serial.println();
    Serial.println(prompt);
    Serial.println("Press ENTER when ready.");

    while (Serial.available() > 0)
        (void)Serial.read();

    while (true)
    {
        if (Serial.available() > 0)
        {
            const char c = static_cast<char>(Serial.read());
            if (c == '\n' || c == '\r')
                break;
        }
        delay(10);
    }

    while (Serial.available() > 0)
        (void)Serial.read();
}

// Calibration stores raw ADC counts in uint16_t; reject failed/out-of-range
// reads before converting them into that storage representation.
static bool readAverage(uint16_t &average)
{
    // A 32-bit accumulator holds 64 full 16-bit ADC samples even on AVR.
    uint32_t sum = 0;
    for (int i = 0; i < kSamples; ++i)
    {
        const int raw = analogRead(kPotPin);
        if (raw < 0 || raw > ArduinoAnalogRead::kFullScale ||
            static_cast<unsigned long>(raw) > 65535UL)
        {
            Serial.println("Acquisition failed or exceeded the ADC range; restart calibration.");
            return false;
        }
        sum += static_cast<uint32_t>(raw);
        delay(kSampleDelayMs);
    }
    average = static_cast<uint16_t>(sum / kSamples);
    return true;
}

static void printResult(const PotCalib &calib)
{
    Serial.println();
    Serial.println("Copy this line into your sketch:");
    Serial.print("PotCalib calib{");
    Serial.print(calib.min);
    Serial.print(", ");
    Serial.print(calib.center);
    Serial.print(", ");
    Serial.print(calib.max);
    Serial.println("};");
}

void setup()
{
    Serial.begin(115200);
    const uint32_t waitStart = millis();
    while (!Serial && (millis() - waitStart) < 3000u)
    {
        delay(10);
    }

    Serial.println();
    Serial.println("PotIO 02 - Pot calibration");
    Serial.println("Move slowly and hold the pot still at each position.");
    Serial.println("This example intentionally avoids ESP32-specific ADC details.");

    waitForEnter("Step 1: move the pot to its minimum end stop.");
    // Raw calibration storage uses 16-bit ADC counts.
    uint16_t minValue = 0;
    if (!readAverage(minValue))
        return;
    Serial.print("min = ");
    Serial.println(minValue);

    waitForEnter("Step 2: move the pot to its physical center.");
    // Raw calibration storage uses 16-bit ADC counts.
    uint16_t centerValue = 0;
    if (!readAverage(centerValue))
        return;
    Serial.print("center = ");
    Serial.println(centerValue);

    waitForEnter("Step 3: move the pot to its maximum end stop.");
    // Raw calibration storage uses 16-bit ADC counts.
    uint16_t maxValue = 0;
    if (!readAverage(maxValue))
        return;
    Serial.print("max = ");
    Serial.println(maxValue);

    const PotCalib calib{minValue, centerValue, maxValue};
    if (!calib.valid_centered())
    {
        Serial.println();
        Serial.println("Calibration rejected: expected min < center < max.");
        Serial.println("Check wiring, pot direction, and whether the center point was captured correctly.");
        Serial.println("Restart calibration after correcting the problem.");
        return;
    }
    printResult(calib);
}

void loop()
{
    // Calibration is a one-shot helper sketch. Nothing is needed in loop().
}
