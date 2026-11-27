/**
 * MIT License
 *
 * @brief Run the calibration example with valid and reversed ADC positions.
 *
 * @file test_calibration_example.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-11-27
 * @copyright Copyright © 2026 Little Man Builds
 */

#include <Arduino.h>
#include <cassert>
#include <cstring>

void setup();

namespace
{
const int *positions;
int position;
int availabilityChecks;
bool copyPrompt;
bool rejection;

void recordLine(const char *line)
{
    if (std::strcmp(line, "Press ENTER when ready.") == 0)
    {
        ++position;
        availabilityChecks = 0;
    }
    copyPrompt |= std::strstr(line, "Copy this line") != nullptr;
    rejection |= std::strstr(line, "Calibration rejected") != nullptr;
}

void capture(const int (&adc)[3], bool valid)
{
    positions = adc;
    position = -1;
    copyPrompt = false;
    rejection = false;
    setup();
    assert(position == 2);
    assert(copyPrompt == valid);
    assert(rejection != valid);
}
}

int main()
{
    Serial.onLine = recordLine;
    Serial.inputAvailable = [] { return ++availabilityChecks == 2 ? 1 : 0; };
    Serial.readInput = []() -> int { return '\n'; };
    potIOAnalogInput = [] {
        assert(position >= 0 && position < 3);
        return positions[position];
    };

    const int valid[] = {100, 2000, 4000};
    const int reversed[] = {4000, 2000, 100};
    capture(valid, true);
    capture(reversed, false);
}
