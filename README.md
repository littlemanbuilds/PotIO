# PotIO

**Zero-allocation potentiometer & joystick I/O for Arduino / embedded C++.**  
Calibrate inputs, apply deadzones, filters, shaping and slew limits — then read a clean **[-1..+1]** signal (or **[0..1]**) with minimal glue code.

> Queues are for streams. Pair PotIO with **SnapshotBus** when you need cross-task “latest state”.

---

## Target support

PotIO's core devices are policy-based C++ and are intended for Arduino-style boards with `analogRead()`. The raw `ArduinoAnalogRead` adapter uses the ADC range for the active core, and you can always provide your own Reader with a correct `kFullScale`.

Tested in this checkout: ESP32-S3 DevKitC-1 as the development/default hardware target, plus an Arduino Uno compile smoke check for the generic `analogRead()` path. Additional compatibility check targets are listed in `platformio.ini`: ESP8266, RP2040 Pico, SAMD MKR Zero, megaAVR Nano Every, Teensy 4.1, and STM32 Blue Pill.

ESP32-only features are kept explicit:

- `ArduinoAnalogReadMilliVolts<FullScaleMv>` and `makePotMilliVolts()` use ESP32 Arduino `analogReadMilliVolts()`.
- `setEsp32PinAttenuation(pin, db)` has an effect on ESP32 Arduino builds only.
- Examples default to ESP32 ADC pins such as GPIO34/GPIO35; use `A0`, `A1`, or your board's ADC pins elsewhere.

Non-Arduino builds can use the device templates with a custom Reader and injected time source, but the Arduino factories are not a portability layer.

---

## What you get

### Devices

- **LinearPot** — 1D pot/axis with centered calibration → **[-1..+1]**
- **Joystick2D** — 2-axis joystick with axial/radial deadzones, optional magnitude + angle
- **ContinuousPot** — “endless” knob with wrap tracking (turn counter + unwrapped angle)
- **SteppedPot** *(new)* — map an analog pot to **N stable positions** (mode knobs / “gears”)

### Policies (mix & match)

- **Filters**: `NoFilter`, `EMAFilter`
- **Rate limits**: `NoRateLimit`, `SlewRate`
- **Shaping**: `ShapeIdentity`, `ShapeCubicExpo`, `ShapeSoftZone`

### Platform helpers

- Arduino reader: `ArduinoAnalogRead`
- ESP32 millivolt reader: `ArduinoAnalogReadMilliVolts<FullScaleMv>`
- ESP32 attenuation helper: `setEsp32PinAttenuation(pin, db)`

---

## Quick start

### Linear pot (raw analogRead)

```cpp
#include <PotIO.h>

auto pot = PotIO::makePotAnalog(A0);   // LinearPot<ArduinoAnalogRead,...>

void loop() {
  pot.update();                        // uses ::millis() on Arduino
  float x = pot.centered();            // [-1..+1]
}
```

### Joystick (KY-023 style)

```cpp
#include <PotIO.h>

auto joy = PotIO::makeJoystickKY023(A0, A1, PotIO::Deadzone::RadialScaled, 0.12f);

void loop() {
  joy.update();
  const auto &s = joy.state();
  // s.x, s.y in [-1..+1]; s.mag in [0..1]; s.angle in radians
}
```

### Continuous knob

```cpp
#include <PotIO.h>

auto knob = PotIO::makeContinuousPot(A0);

void loop() {
  knob.update();
  float phase = knob.phase01();            // [0..1)
  float turns = knob.turns_raw();          // signed
  float deg   = knob.angleUnwrappedDeg();  // unwrapped degrees since start
}
```

### Stepped knob (e.g., 5 positions)

```cpp
#include <PotIO.h>

// Steps is a compile-time constant.
auto gear = PotIO::makeSteppedPot<5>(A0);

void loop() {
  gear.update();
  if (gear.state().changed) {
    // step is 0..4
  }
}
```

---

## Calibration

Use `PotCalib{min, center, max}` when a physical center matters (joysticks / centered pots).

If calibration is invalid (or left default), PotIO falls back to simple normalization using the reader’s
full-scale (either `Reader::kFullScale` or `POTIO_DEFAULT_FULLSCALE`).

---

## Time source injection (world-class ergonomics)

All devices support:

- `update()` → uses internal time source (`::millis()` on Arduino)
- `update(now_ms)` → computes dt internally (wrap-safe, first-call guarded)
- `update(now_ms, dt_s)` → advanced explicit time step

You can inject a time function:

```cpp
uint32_t now_ms() { return /* your clock */; }
pot.setTimeSource(now_ms);
```

---

## Changing configuration at runtime

Every device supports `setConfig(config)` to replace configuration while preserving current output history. If you are changing calibration, filter strength, slew rate, or wrap hysteresis and want a clean restart, pass `true`:

```cpp
pot.setConfig(cfg, true);   // reset filter/rate/timing state before the next sample
```

You can also call `resetState()` directly. `ContinuousPot` still has `resetTurns()` when you only want to zero the turn counter.

---

## ADC full-scale and portability

Different Arduino cores use different ADC ranges (and some allow changing resolution at runtime).

PotIO determines full-scale in this order:

1. `Reader::kFullScale` (best, explicit)
2. `POTIO_DEFAULT_FULLSCALE` (compile-time default)

Arduino builds default to:

- ESP32: 4095
- others: 1023

Override for your project:

```cpp
#define POTIO_ARDUINO_ADC_FULLSCALE 4095
#include <PotIO.h>
```

Or provide a custom Reader with `kFullScale`.

### ESP32 ADC notes

For ESP32 projects using Wi-Fi, prefer ADC1 pins. ADC2 pins are shared with the Wi-Fi hardware on many ESP32 variants and can behave differently once Wi-Fi is active.

Set attenuation to match the voltage you actually feed into the ADC. `11 dB` is a common starting point for a 3.3 V pot, but it is not a calibration substitute.

For best results, measure real values from your build:

- `min`: pot at one end stop
- `center`: physical center, if the control has one
- `max`: pot at the other end stop

Then use those observed values in `PotCalib{min, center, max}`. With `makePotMilliVolts()`, those values are millivolts; with `makePotAnalog()`, they are raw ADC counts.

---

## SnapshotBus compatibility (recommended pattern)

PotIO devices are intentionally single-writer objects. For RTOS / multicore, publish a POD **Frame**
to a `SnapshotBus`, and let other tasks read a consistent snapshot.

Every device provides `frame()` which returns a POD copy suitable for SnapshotBus.

---

## File layout

- `PotIO.h` — umbrella include
- `PotIO_*` — policies, helpers, Arduino adapters
- `src/devices/*` — devices
- `src/devices/helpers/*` — helper presets / facades

---

## License

MIT
