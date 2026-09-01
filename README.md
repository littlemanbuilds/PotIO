# PotIO

PotIO is a small, allocation-free Arduino library for turning potentiometers and analog joysticks into calibrated, normalized input data that is easier for an application to use safely and consistently.

It supports linear potentiometers, two-axis joysticks, cyclic/continuous analog controls, and analog controls used as stable discrete selectors. PotIO owns **input acquisition and interpretation**. It deliberately does not own motor control, vehicle policy, task transport, persistence, or application safety decisions.

- **Current version:** v1.1.0
- **Primary development target:** ESP32-S3 / Arduino
- **Language level:** C++11

---

## Contents

- [Why PotIO exists](#why-potio-exists)
- [Design boundaries](#design-boundaries)
- [Installation](#installation)
- [Beginner's guide](#beginners-guide)
- [The v1.1 validity model](#the-v11-validity-model)
- [Calibration](#calibration)
- [LinearPot](#linearpot)
- [Joystick2D](#joystick2d)
- [ContinuousPot](#continuouspot)
- [SteppedPot](#steppedpot)
- [Filters, shaping, and rate limiting](#filters-shaping-and-rate-limiting)
- [Jitter tools](#jitter-tools)
- [Custom readers](#custom-readers)
- [ESP32 ADC notes](#esp32-adc-notes)
- [Using PotIO in a larger control project](#using-potio-in-a-larger-control-project)
- [Examples](#examples)
- [Testing and validation](#testing-and-validation)
- [Configuration and public API](#configuration-and-public-api)
- [Migration from v1.0.0](#migration-from-v100)
- [Repository structure](#repository-structure)
- [Deliberate limitations](#deliberate-limitations)
- [Version history](#version-history)
- [License](#license)

---

## Why PotIO exists

Reading a potentiometer with `analogRead()` is easy. Building a reusable analog-input layer is less simple once a real project needs:

- measured min / center / max calibration;
- normalized `0..1` or centered `-1..1` values;
- deadzones around a joystick's physical center;
- filtering without mixing filter state with later slew limiting;
- response shaping;
- rate limiting;
- explicit acquisition failure handling;
- timing-gap detection;
- stable analog-to-step conversion;
- cyclic analog wrap tracking;
- data that can be published cleanly to another task or subsystem.

PotIO packages those concerns into reusable devices while keeping application meaning outside the library. A PotIO value can become steering, throttle, trim, menu input, volume, accessibility control, or something else entirely. PotIO does not decide which.

---

## Design boundaries

### PotIO owns

PotIO is responsible for:

- calling a supplied analog reader;
- validating whether the acquisition succeeded;
- validating the reader's declared full-scale;
- validating or deliberately falling back from calibration;
- converting raw values into normalized values;
- deadzone and joystick geometry handling;
- optional shaping, filtering, and slew limiting;
- timing validation for update steps;
- continuous-pot wrap plausibility;
- stepped-pot hysteresis and change sequencing;
- retaining the last good output when a new acquisition is invalid;
- exposing small copyable state frames.

### The host application owns

The application remains responsible for:

- wiring and ADC voltage limits;
- choosing ADC pins appropriate to the board;
- deciding whether a control can affect safety or movement;
- deciding freshness limits for a published frame;
- persistence of calibration values;
- task/thread transport;
- actuator behavior;
- emergency stop and containment;
- deciding what should happen when `state().status.valid == false`.

PotIO has no dependency on SnapshotBus, SafetyCore, NVMKit, SteerCore, or another Little Man Builds library.

---

## Installation

### Arduino IDE

1. Download the PotIO release ZIP.
2. Open **Sketch → Include Library → Add .ZIP Library...**.
3. Select the PotIO ZIP.
4. Open **File → Examples → PotIO → 01_LinearPotBasic**.

### PlatformIO

Add PotIO to the project's library dependencies, or place the library in the project's `lib/` directory.

Then include:

```cpp
#include <PotIO.h>
```

### Manual installation

Copy the `PotIO` folder into your Arduino libraries directory. The folder should contain `src/`, `examples/`, `library.properties`, and this README.

---

## Supported targets

PotIO is portable across the architectures declared by `library.properties`. ESP32-S3 is the normal local reference target; CI keeps one representative compile target for AVR, megaAVR, SAM, SAMD, ESP32, ESP8266, STM32, Teensy and RP2040. Hardware-specific ADC behaviour still has to be validated on the actual board and wiring used by the application.

---

## Beginner's guide

This section is intentionally the shortest route from a potentiometer on the bench to useful data in Serial Monitor.

### 1. Wire one potentiometer

A normal three-terminal potentiometer has:

- one outer terminal to **3.3 V**;
- the other outer terminal to **GND**;
- the center wiper to an ADC-capable GPIO.

For an ESP32-S3 DevKitC-1, the examples use **GPIO4** as a simple demonstration ADC input. Check your exact board before copying a pin assignment.

Do not put 5 V onto an ESP32 ADC input.

### 2. Start with the basic factory

```cpp
#include <PotIO.h>
#include <Arduino.h>

static constexpr uint8_t kPotPin = 4;
auto pot = PotIO::makePotAnalog(kPotPin);

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    pot.update();

    const auto s = pot.state();
    if (s.status.valid)
    {
        Serial.println(s.centered, 3);
    }

    delay(20);
}
```

The important beginner value is usually:

- `0.0` near the calibrated center;
- `-1.0` towards one end;
- `+1.0` towards the other end.

### 3. Understand the three useful numbers

A `LinearPot` state contains:

- `raw01` — raw ADC position normalized to `0..1`;
- `calib01` — position after calibration, also `0..1`;
- `centered` — final centered output in `-1..1` after shaping/filtering/rate limiting.

For many first projects, `centered` is the value you actually want.

### 4. Calibrate the real hardware

Run **02_PotCalibration** and measure the real minimum, physical center, and maximum.

Then use those values:

```cpp
PotIO::PotCalib calib{180, 2050, 3890};
auto pot = PotIO::makePotAnalog(kPotPin, calib);
```

Calibration matters because real pots usually do not reach perfect ADC rails and their mechanical center is rarely the mathematical midpoint.

### 5. Always understand validity before using PotIO for control

For a display knob, ignoring a rare invalid sample may be harmless. For steering, throttle, or another safety-critical input, it is not.

PotIO v1.1.0 never converts a failed read into a valid full-low/full-negative command. Instead:

```cpp
pot.update();
const auto s = pot.state();

if (!s.status.valid)
{
    // Do not treat s.centered as a fresh command.
    // The numeric value is the retained last-good output.
    return;
}
```

This distinction is central to the v1.1 API.

### 6. Move to a joystick only after one axis makes sense

A KY-023 style joystick is simply two analog axes. Start with:

```cpp
auto joystick = PotIO::makeJoystickKY023<>(4, 5);
```

Then:

```cpp
joystick.update();
const auto s = joystick.state();

if (s.status.valid)
{
    Serial.print(s.x);
    Serial.print(' ');
    Serial.println(s.y);
}
```

The default deadzone is radial-scaled. The default output geometry remains square, so X and Y retain independent full-scale authority.

### 7. Use strict calibration for safety-critical controls

The beginner factories remain permissive because that makes first use simple. A control-system configuration can require real calibration:

```cpp
using SteeringInput = PotIO::LinearPot<PotIO::ArduinoAnalogRead>;

SteeringInput::Config cfg;
cfg.reader = PotIO::ArduinoAnalogRead(4);
cfg.calib = PotIO::PotCalib{180, 2050, 3890};
cfg.calibration_policy = PotIO::CalibrationPolicy::RequireValid;

SteeringInput steering(cfg);
```

If calibration is missing, malformed, outside the reader range, or has insufficient span, the update is invalid instead of silently creating a plausible command.

---

## The v1.1 validity model

Every device state now contains a `SampleStatus` named `status`.

```cpp
struct SampleStatus
{
    bool valid;
    bool has_value;
    bool calibration_valid;
    ReadError error;
    uint16_t quality;
    uint32_t sample_ms;
    uint32_t sequence;
};
```

### `valid`

`true` means the **latest update attempt** produced a fresh sample and the device pipeline accepted it.

### `has_value`

`true` means the device has acquired at least one valid sample at some point. If a later update fails, the numeric output is retained but `valid` becomes false.

This prevents the failure mode where an ADC error becomes a believable extreme command.

### `sample_ms`

Timestamp of the most recent successful acquisition. It does not move forward on a failed read.

The application's transport layer can add its own publication timestamp, but it should not confuse publication freshness with acquisition freshness.

### `sequence`

Monotonic count of successful samples. It wraps naturally as a `uint32_t`.

This is useful when a host application wants a source-level freshness or diagnostic signal without adding a dependency to PotIO.

### `calibration_valid`

Reports whether the configured calibration is genuinely valid for that reader's full-scale.

In `PermissiveDefault` mode, the sample can still be valid while `calibration_valid == false`; `QualityCalibrationFallback` is then set.

### `ReadError`

The latest invalid update reports one primary reason:

| Error | Meaning |
|---|---|
| `None` | Update succeeded. |
| `NoSample` | No successful sample has been acquired yet. |
| `ReaderFailure` | Reader explicitly failed or returned a negative integer. |
| `OutOfRange` | Reader returned above its declared full-scale. |
| `CalibrationInvalid` | Strict calibration policy rejected calibration. |
| `InvalidTiming` | `dt_s` was negative, NaN, or infinite. |
| `TimingGap` | `dt_s` exceeded `max_dt_s`. |
| `InvalidConfiguration` | A built-in policy/device setting was invalid. |
| `Discontinuity` | ContinuousPot motion could not be tracked unambiguously. |

### What happens on an invalid update?

PotIO deliberately does **not** advance:

- filter history;
- slew/rate history;
- joystick partial-axis state;
- continuous-pot wrap state;
- stepped-pot step transitions;
- successful sample timestamp;
- successful sample sequence.

The public numeric output stays at its last-good value.

`InvalidSamplePolicy::ResetProcessing` is available when an application explicitly wants filter/rate history reseeded after invalid acquisition. The default is `HoldState`.

---

## Calibration

`PotCalib` stores:

```cpp
PotIO::PotCalib calib{minimum, center, maximum};
```

A centered calibration is valid when:

```text
min < center < max
```

v1.1 additionally validates that:

- `max` does not exceed the reader's declared full-scale;
- each side of center meets `min_calibration_span`;
- strict users can require the calibration instead of falling back.

### Permissive calibration

Default behavior:

```cpp
cfg.calibration_policy = PotIO::CalibrationPolicy::PermissiveDefault;
```

If calibration is invalid, PotIO maps against the reader's declared full-scale and sets the calibration fallback quality flag.

This keeps simple sketches easy to start.

### Strict calibration

For a control input:

```cpp
cfg.calibration_policy = PotIO::CalibrationPolicy::RequireValid;
```

An invalid calibration then produces `ReadError::CalibrationInvalid` and no pipeline state advances.

### Calibration changes at runtime

When `setConfig()` changes calibration, PotIO automatically re-seeds processing history so filter state from the old coordinate system is not mixed with the new one.

For `ContinuousPot`, changing calibration also re-arms phase synchronization while preserving the current requested turn reference.

---

## LinearPot

`LinearPot` is the general one-axis device.

Pipeline:

```text
reader
  ↓
acquisition validation
  ↓
calibration / normalization
  ↓
center to -1..1
  ↓
shape
  ↓
filter
  ↓
rate limit
  ↓
State
```

The filter and rate limiter now maintain **independent stage history**. This matters when both are enabled: the filter uses the previous filtered value, while the slew limiter uses the previous final output. v1.0.0 incorrectly fed the slew-limited output back into the filter.

Common accessors:

```cpp
pot.update();
pot.valid();
pot.calib01();
pot.centered();
pot.state();
pot.frame();
pot.fullScale();
```

---

## Joystick2D

`Joystick2D` treats X and Y as one acquisition. If either axis fails, neither axis advances.

### Deadzone policies

PotIO supports:

- `Deadzone::None`
- `Deadzone::Axial`
- `Deadzone::AxialScaled`
- `Deadzone::Radial`
- `Deadzone::RadialScaled`

`AxialScaled` was added in v1.1.0. It removes the discontinuity of a simple axial deadzone by remapping the range outside the threshold back to full travel.

### Radial scaling and square-corner inputs

Two independently normalized axes can produce a vector magnitude up to approximately `sqrt(2)`.

v1.1.0 prevents `RadialScaled` from accidentally **amplifying** a square-corner vector whose magnitude is already above one.

### Geometry policies

The geometry policy is explicit:

```cpp
cfg.geometry = PotIO::JoystickGeometry::Square;
```

Available choices:

- `Square` — X and Y retain independent full-scale authority;
- `MagnitudeClamp` — vector magnitude is clamped to one while preserving angle;
- `SquareToCircle` — smooth square-domain to circle mapping.

Independent X/Y calibration already handles unequal physical axis spans before the geometry stage.

### Magnitude and angle

When enabled, `mag` is reported in `0..1`.

At the exact center, an angle of zero is numerically possible but semantically meaningless. v1.1 therefore adds:

```cpp
state.angle_valid
joystick.angleValid()
```

Use the angle only when it is valid.

---

## ContinuousPot

`ContinuousPot` is for a **cyclic analog signal** where moving across the `0/1` phase boundary represents another turn.

A normal mechanical potentiometer with hard end stops does not become endless just because this class is used.

### Wrap tracking

The library tracks:

- current normalized phase;
- signed turn count;
- unwrapped angle;
- whether turn tracking remains valid.

### Plausibility limits

v1.1.0 adds two checks:

```cpp
cfg.max_phase_delta = 0.45f;
cfg.max_turns_per_s = 8.0f;
```

If consecutive samples imply movement that cannot be interpreted safely, the update reports `ReadError::Discontinuity` and `turns_valid` becomes false.

The plausibility interval is measured from the last **successful phase sample**, not merely the last update attempt. A failed ADC read therefore cannot make a long sampling gap look artificially short. PotIO also rejects a gap when the configured maximum speed could cover half a turn between valid phase samples, because a hidden wrap would then be mathematically ambiguous.

A large skipped movement can otherwise be indistinguishable from a legitimate wrap in sampled cyclic data.

### Explicit resynchronization

After a discontinuity:

```cpp
knob.resynchronizeTurns(known_turn_count);
```

The next valid sample seeds phase tracking and cannot create a false wrap.

Check:

```cpp
knob.turnsValid();
```

before treating an unwrapped angle as safe to use.

---

## SteppedPot

`SteppedPot<N>` turns one analog input into `N` stable positions with hysteresis.

Typical uses:

- mode selectors;
- profile knobs;
- menu selection;
- coarse trim positions.

### `changed`

`state.changed` remains a convenient one-update pulse.

### `change_sequence`

A one-update boolean can disappear if a latest-state frame is overwritten before a slower consumer sees it. v1.1 adds:

```cpp
state.change_sequence
pot.changeSequence()
```

The counter increments on every accepted step transition. A consumer can compare the last sequence it observed and know that a change happened even if it missed the pulse.

If **every individual transition** must be delivered, use an event queue or similar event transport in the host application instead of relying only on a latest-state frame.

---

## Filters, shaping, and rate limiting

### Filters

Built-ins:

- `NoFilter`
- `EMAFilter`

Example:

```cpp
cfg.filter.alpha = 0.12f;
```

Built-in policy configuration is validated. NaN and out-of-range parameters are not silently allowed to corrupt a control pipeline.

### Shaping

Built-ins:

- `ShapeIdentity`
- `ShapeCubicExpo`
- `ShapeSoftZone`

Shaping happens before filtering and slew limiting.

### Rate limiting

Built-ins:

- `NoRateLimit`
- `SlewRate`

Example:

```cpp
cfg.rate.units_per_s = 3.0f;
```

`SlewRate` requires a finite non-negative rate.

### Timing validation

Each device has:

```cpp
cfg.max_dt_s = 0.5f;
```

A negative, NaN, or infinite explicit `dt_s` is rejected as `InvalidTiming`.

An update gap larger than `max_dt_s` is rejected as `TimingGap`. PotIO holds the last good output instead of applying one very large integration step through a slew limiter.

The built-in timestamp path remains wrap-safe across `uint32_t millis()` rollover.

---

## Jitter tools

v1.0.0's `JitterStats` accumulated lifetime min/max, so one startup outlier could dominate the deadzone estimate forever.

v1.1.0 makes the default `JitterStats` a 32-sample rolling window:

```cpp
PotIO::JitterStats jitter;

jitter.observe(rawCounts);
const uint16_t pp = jitter.peak_to_peak();
```

A custom window is available without dynamic allocation:

```cpp
PotIO::RollingJitterStats<64> jitter;
```

Then:

```cpp
const float dz = PotIO::suggest_deadzone_from_jitter(
    jitter.peak_to_peak(),
    4095,
    5);
```

This remains a diagnostic/tuning aid, not an automatic safety policy.

---

## Custom readers

### Existing integer readers

Any callable that returns an integer still works:

```cpp
struct MyReader
{
    static constexpr int kFullScale = 4095;

    int operator()() const
    {
        return readMyAdc();
    }
};
```

For backward compatibility, a negative result now means **read failure** instead of being clamped to zero.

### Explicit `RawSample` readers

A reader that has richer failure knowledge can return:

```cpp
PotIO::RawSample operator()() const
{
    if (!hardwareReady())
        return PotIO::RawSample::failure(PotIO::ReadError::ReaderFailure);

    return PotIO::RawSample::success(readMyAdc());
}
```

This keeps acquisition failure separate from the real ADC value `0`.

### Reader full-scale

A custom reader should declare:

```cpp
static constexpr int kFullScale = 4095;
```

If it does not, PotIO uses `POTIO_DEFAULT_FULLSCALE`.

---

## ESP32 ADC notes

ESP32 ADC behavior is board/core specific. PotIO deliberately avoids pretending every board has the same ADC setup.

### Raw count reader

```cpp
PotIO::ArduinoAnalogRead
```

uses Arduino `analogRead()`.

### Millivolt reader

On ESP32 only:

```cpp
PotIO::ArduinoAnalogReadMilliVolts<3300>
```

uses `analogReadMilliVolts()`.

The uncalibrated millivolt factory now seeds an ideal `0 / FullScaleMv/2 / FullScaleMv` calibration so its declared coordinate system is internally consistent.

### Attenuation helper

```cpp
PotIO::setEsp32PinAttenuation(pin, 11);
```

is an ESP32 convenience and a no-op elsewhere.

Always verify:

- ADC-capable pins on the exact MCU/board;
- allowed input voltage;
- attenuation behavior;
- ADC resolution;
- Wi-Fi/ADC restrictions on the relevant ESP32 family/core;
- source impedance and noise in the real circuit.

---

## Using PotIO in a larger control project

PotIO intentionally emits input state rather than vehicle policy.

A robust integration normally looks like:

```text
physical ADC
   ↓
PotIO reader/device
   ↓
PotIO State + SampleStatus
   ↓
application adapter adds project/source context
   ↓
latest-state transport
   ↓
authority / safety policy
   ↓
actuator command
```

For a joystick that can affect movement or safety, the project adapter should normally propagate at least:

- `status.valid`;
- `status.sample_ms`;
- `status.sequence`;
- calibration validity;
- project publication timestamp;
- source identity/authority;
- any application freshness timeout.

A latest-state transport is appropriate for "what is the joystick position now?". It is not automatically appropriate for guaranteed delivery of every discrete event. `SteppedPot::change_sequence` helps detect missed changes, but a queue remains the right tool when every event matters.

---

## API reference and configuration

### Package version macros

> **Compatibility note:** the generic `LIBRARY_VERSION*` aliases considered during v1.1.0 development are intentionally absent from the release because those global macro names collide when multiple libraries are included together. Use `POTIO_VERSION*`.

```cpp
POTIO_VERSION
POTIO_VERSION_MAJOR
POTIO_VERSION_MINOR
POTIO_VERSION_PATCH
```

PotIO deliberately uses package-specific version macros so it can coexist safely with other libraries in the same application.

### Main public types

```text
PotCalib
RawSample
SampleStatus
ReadError
CalibrationPolicy
InvalidSamplePolicy
Deadzone
JoystickGeometry
LinearPot
Joystick2D
ContinuousPot
SteppedPot
AnalogStick
RollingJitterStats
JitterStats
```

### Arduino readers

```text
ArduinoAnalogRead
ArduinoAnalogReadMilliVolts
```

### Built-in policies

```text
NoFilter
EMAFilter
NoRateLimit
SlewRate
ShapeIdentity
ShapeCubicExpo
ShapeSoftZone
```

### Main factories

```text
makePotAnalog()
makePotMilliVolts()
makeContinuousPot()
makeJoystickKY023()
makeSteppedPot()
makeKnobSticky()
makeKnobSmooth()
makeKnobJog()
```

### Compile-time ADC defaults

`POTIO_ARDUINO_ADC_FULLSCALE` can override the default Arduino raw full-scale.

`POTIO_DEFAULT_FULLSCALE` is used when a custom reader does not declare `kFullScale`.

For reusable code, a reader-specific `kFullScale` is normally clearer than relying on a global default.

---

## Examples

PotIO keeps the public examples progressive rather than turning every sketch into a feature demonstration.

| Example | Purpose |
|---|---|
| `01_LinearPotBasic` | First potentiometer read and normalized output. |
| `02_PotCalibration` | Measure real min / center / max. |
| `03_ContinuousPotAdvanced` | Phase, wrap tracking, turns, and unwrapped angle. |
| `04_Joystick2DKY023` | Two-axis joystick and radial deadzone. |
| `05_ESP32ADC` | ESP32 attenuation and millivolt reading. |
| `06_PotShaping` | Compare response curves and slew limiting. |
| `07_PotSteering` | Steering-style normalized input without actuator control. |
| `08_SteppedPotModeKnob` | Stable analog mode selector with hysteresis. |

The examples use ESP32-S3-friendly ADC choices where possible and retain generic Arduino fallbacks.

---

## Testing and validation

PotIO v1.1.0 moves away from the old `platformio.ci.ini` pattern and uses a dedicated `test/` structure plus CI workflow.

### Complete host check

Run:

```bash
./test/run_host_checks.sh
```

This is the quickest local release check. It runs the native tests, sanitizer pass, example syntax checks, and release-contract checks.

### Native deterministic tests

Run:

```bash
./test/run_native_tests.sh
```

The suite covers:

- integer-reader failure and recovery contract;
- explicit `RawSample` failure;
- out-of-range acquisition;
- permissive and strict calibration;
- calibration full-scale and minimum-span validation;
- independent filter and slew state;
- negative / NaN / infinite `dt_s`;
- excessive scheduling gaps;
- invalid built-in policy configuration;
- non-finite custom processing output rejection;
- axial-scaled deadzone continuity;
- radial square-corner gain prevention;
- joystick geometry policies;
- neutral angle validity;
- atomic two-axis failure behavior;
- ContinuousPot wrap tracking;
- phase discontinuity detection and explicit re-sync;
- velocity plausibility;
- SteppedPot hysteresis;
- SteppedPot durable change sequencing;
- rolling jitter behavior;
- `uint32_t` time wraparound.

The native suite is compiled as C++11 with strict warnings enabled.

### Example syntax checks

Run:

```bash
./test/check_examples_syntax.sh
```

This compiles every public sketch against the real PotIO headers using a small ESP32-S3-oriented Arduino host stub.

### Release-contract checks

Run:

```bash
./test/check_release_contracts.sh
```

This checks version consistency, package metadata, example manifests, Doxygen/LMB headers, required test structure, and unwanted generated files.

### Hardware / PlatformIO compile matrix

The repository workflow is configured to compile the portable public API across representative Arduino targets including:

- ESP32-S3 DevKitC-1;
- classic ESP32;
- ESP8266;
- RP2040;
- SAMD;
- Arduino Due / SAM;
- Nano Every;
- AVR Uno;
- Teensy 4.1;
- STM32 Blue Pill.

ESP32-S3 is the primary development target. The other boards are compile-compatibility targets; they do not imply identical ADC electrical behavior or runtime quality.

Host tests cannot validate real ADC noise, board attenuation, source impedance, or analog wiring. Those remain hardware validation items.

---

## Migration from v1.0.0

v1.1.0 is a **minor** release: existing normal factories, device templates, `update()`, `state()`, `frame()`, and primary numeric accessors remain available.

Important behavioral changes are intentional defect fixes.

### Negative reader results

- **v1.0.0:** negative value was clamped to zero and could become full-negative output.
- **v1.1.0:** negative value means `ReaderFailure`; numeric output remains last-good and `status.valid == false`.

### Invalid calibration

Default behavior remains permissive, but the state now tells you that fallback occurred. Authoritative applications can opt into `CalibrationPolicy::RequireValid`.

### Filter + slew pipeline

The two stages now use independent history. Tuned behavior that unknowingly depended on the v1.0 coupling may respond slightly differently and should be bench checked.

### Timing gaps

Very large or invalid explicit `dt_s` no longer passes into the rate limiter. Adjust `max_dt_s` if your legitimate update cadence is slower than the default 0.5 s.

### Joystick deadzones

`RadialScaled` no longer amplifies square-domain vectors that are already outside the unit circle. `AxialScaled` and explicit geometry policies are new.

### Joystick angle

`angle` still returns zero at neutral for source compatibility, but `angle_valid` / `angleValid()` now states whether that number has semantic meaning.

### ContinuousPot

Wrap tracking now has plausibility limits. If a discontinuity is detected, call `resynchronizeTurns()` after the application decides the correct turn reference.

### SteppedPot

`changed` remains available. `change_sequence` is new and is preferred when the state crosses a latest-state transport boundary.

### JitterStats

`JitterStats` now represents the most recent 32 samples instead of lifetime min/max. This is intentionally more useful for current noise estimation.

### PlatformIO CI file

The legacy `platformio.ci.ini` is removed. Tests live under `test/`, and the repository CI workflow owns the multi-board compile matrix.

---

## Deliberate limitations

PotIO deliberately does not:

- allocate dynamically;
- create RTOS tasks;
- synchronize access between tasks/cores;
- save calibration to non-volatile memory;
- decide whether stale/invalid input should stop a vehicle;
- drive motors or steering actuators;
- guarantee delivery of every event;
- compensate automatically for ADC electrical non-linearity;
- infer whether a physical control is safe to use.

Those boundaries keep the library independently useful and prevent it from becoming coupled to one application architecture.

---

## Repository structure

```text
PotIO/
├── .github/workflows/ci.yml
├── examples/
│   ├── 01_LinearPotBasic/
│   ├── 02_PotCalibration/
│   ├── 03_ContinuousPotAdvanced/
│   ├── 04_Joystick2DKY023/
│   ├── 05_ESP32ADC/
│   ├── 06_PotShaping/
│   ├── 07_PotSteering/
│   └── 08_SteppedPotModeKnob/
├── src/
│   ├── devices/
│   ├── PotIO.h
│   ├── PotIO_Arduino.h
│   ├── PotIO_Compatibility.h
│   ├── PotIO_Detail.h
│   ├── PotIO_Factory.h
│   ├── PotIO_Filters.h
│   ├── PotIO_JitterTools.h
│   ├── PotIO_RateLimit.h
│   ├── PotIO_Shaping.h
│   └── PotIO_Types.h
├── test/
│   ├── native/
│   ├── portable_compile/
│   ├── support/
│   ├── check_examples_syntax.sh
│   ├── check_release_contracts.sh
│   ├── run_host_checks.sh
│   ├── run_sanitizers.sh
│   └── run_native_tests.sh
├── CHANGELOG.md
├── RELEASE_CHECKLIST.md
├── keywords.txt
├── library.json
├── library.properties
└── platformio.ini
```

---

## Version history

See [CHANGELOG.md](CHANGELOG.md) for detailed release notes.

- **v1.1.0** — explicit acquisition validity, strict calibration option, corrected processing-stage state, timing validation, joystick geometry/deadzone improvements, ContinuousPot plausibility, durable SteppedPot change sequencing, rolling jitter, and full test/release structure.
- **v1.0.0** — initial public release.

---

## License

PotIO is released under the **MIT License**. See [LICENSE](LICENSE).

Copyright © 2026 Little Man Builds (Darren Osborne).
