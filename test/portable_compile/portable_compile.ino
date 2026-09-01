/**
 * MIT License
 *
 * @brief Minimal portable compile smoke test for the public PotIO API.
 *
 * @file portable_compile.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright © 2026 Little Man Builds
 */
#include <PotIO.h>

#if defined(ARDUINO_ARCH_ESP32)
static constexpr uint8_t kPin = 4;
#else
static constexpr uint8_t kPin = A0;
#endif

auto potioPortablePot = PotIO::makePotAnalog(kPin);

void setup()
{
    potioPortablePot.update();
}

void loop()
{
    potioPortablePot.update();
}
