/**
 * MIT License
 *
 * @file 02_PotCalibration.ino
 * @brief Measure real min, center, and max values for PotIO::PotCalib.
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
 *   4. Copy the printed PotIO::PotCalib into your real sketch.
 */

#include <PotIO.h>

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
// GPIO4 is used here because it is a simple ADC-capable example pin on
// ESP32-S3 boards. If your board routes ADC differently, change this pin.
static constexpr uint8_t kPotPin = 4;
#else
static constexpr uint8_t kPotPin = A0;
#endif

// Averaging a small number of samples makes the captured calibration point more
// repeatable without turning this into a complicated data-logging sketch.
static constexpr uint16_t kSamples = 64;
static constexpr uint16_t kSampleDelayMs = 3;

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

static uint16_t readAverage()
{
    uint32_t sum = 0;
    for (uint16_t i = 0; i < kSamples; ++i)
    {
        sum += static_cast<uint32_t>(analogRead(kPotPin));
        delay(kSampleDelayMs);
    }
    return static_cast<uint16_t>(sum / kSamples);
}

static void printResult(const PotIO::PotCalib &calib)
{
    Serial.println();
    Serial.println("Copy this line into your sketch:");
    Serial.print("PotIO::PotCalib calib{");
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
    const uint16_t minValue = readAverage();
    Serial.print("min = ");
    Serial.println(minValue);

    waitForEnter("Step 2: move the pot to its physical center.");
    const uint16_t centerValue = readAverage();
    Serial.print("center = ");
    Serial.println(centerValue);

    waitForEnter("Step 3: move the pot to its maximum end stop.");
    const uint16_t maxValue = readAverage();
    Serial.print("max = ");
    Serial.println(maxValue);

    const PotIO::PotCalib calib{minValue, centerValue, maxValue};
    printResult(calib);

    if (!calib.valid_centered())
    {
        Serial.println();
        Serial.println("Warning: expected min < center < max.");
        Serial.println("Check wiring, pot direction, and whether the center point was captured correctly.");
    }
}

void loop()
{
    // Calibration is a one-shot helper sketch. Nothing is needed in loop().
}
