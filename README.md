# PotIO

**Practical potentiometer, joystick, and analog-control input handling for Arduino and embedded projects.**

PotIO turns noisy ADC readings into clean, useful control values. It helps you read linear potentiometers, analog joysticks, continuous knobs, stepped mode knobs, steering-style pots, pedals, sliders, and trim controls with calibration, deadzones, filtering, shaping, slew limiting, and SnapshotBus-friendly output frames.

PotIO is written in the Little Man Builds style: simple enough for Arduino beginners, but structured enough for real ESP32-S3 robotics, ride-on vehicle, control panel, and actuator-feedback projects.

> PotIO reads and shapes analog input. It does not drive motors, run PID, decide steering authority, or enforce vehicle safety policy.

---

## What PotIO is for

Use PotIO when you have an analog input such as:

- a steering potentiometer
- a throttle pedal or slider
- a KY-023 style joystick
- a trim knob
- a mode selector knob
- a cyclic / endless-feeling rotary control
- an analog input that needs smoothing, calibration, or deadzone handling

PotIO gives you project-friendly values such as:

- `0.0 .. 1.0` for absolute travel
- `-1.0 .. +1.0` for centered controls
- discrete step numbers for mode knobs
- joystick `x`, `y`, magnitude, and angle
- compact `Frame` structs for SnapshotBus-style latest-state publishing

---

## What PotIO deliberately does not do

PotIO does **not**:

- drive motors
- apply PWM
- implement PID
- decide steering authority
- decide vehicle safety behavior
- read RC receivers
- own joystick/app/remote-control policy
- replace a proper control loop

For a ride-on vehicle project, PotIO might read a pedal, joystick, trim knob, or steering feedback pot. The main project, SteerCore, or another control layer decides what to do with that value.

---

## Supported platforms

PotIO is intended for Arduino-compatible boards with `analogRead()` support.

The intended Arduino architecture list is:

```text
avr, megaavr, sam, samd, esp32, esp8266, stm32, teensy, rp2040
```

The matching PlatformIO platform list is:

```text
atmelavr, atmelmegaavr, atmelsam, espressif32, espressif8266, ststm32, teensy, raspberrypi
```

ESP32-S3 is the primary development target for current Little Man Builds projects. The general examples therefore use ESP32-S3-compatible ADC pins when compiled for ESP32, while still falling back to normal Arduino analog pins on other boards.

### ESP32-only features

These helpers are ESP32-specific:

- `ArduinoAnalogReadMilliVolts<FullScaleMv>`
- `makePotMilliVolts()`
- `setEsp32PinAttenuation(pin, atten_db)`

The normal `ArduinoAnalogRead` path uses ordinary `analogRead()` and should remain portable where the Arduino core provides a working ADC API.

---

## Installation

### Arduino IDE

1. Download the PotIO ZIP from the project release page.
2. Open Arduino IDE.
3. Go to **Sketch → Include Library → Add .ZIP Library...**
4. Select the PotIO ZIP.
5. Open one of the numbered examples from **File → Examples → PotIO**.

### PlatformIO

Once published, add PotIO as a dependency in `platformio.ini`. During development, place the library folder in your project `lib/` directory.

```text
YourProject/
  platformio.ini
  src/
    main.cpp
  lib/
    PotIO/
      src/
      examples/
      library.json
```

---

## Wiring basics

A normal potentiometer has three pins:

| Pot pin      | Connect to                            |
| ------------ | ------------------------------------- |
| One end      | GND                                   |
| Middle/wiper | ADC pin                               |
| Other end    | 3.3 V or 5 V, depending on your board |

Important: the ADC pin must never receive more voltage than the microcontroller allows.

- ESP32 / ESP32-S3 ADC pins are **not 5 V tolerant**.
- Many classic Arduino boards use 5 V ADC inputs.
- Always check your board before wiring a 5 V pot to a 3.3 V microcontroller.

### ESP32-S3 example pins

The examples use these ESP32-S3-friendly ADC pins when `ARDUINO_ARCH_ESP32` is defined:

| Use        | ESP32-S3 example pin |
| ---------- | -------------------- |
| Single pot | GPIO4                |
| Joystick X | GPIO4                |
| Joystick Y | GPIO5                |

Older ESP32 examples often use GPIO34/GPIO35. Those are not a good generic choice for ESP32-S3 projects, so PotIO examples use GPIO4/GPIO5 instead.

---

## Quick start: linear potentiometer

```cpp
#include <Arduino.h>
#include <PotIO.h>

#if defined(ARDUINO_ARCH_ESP32)
// GPIO4 is used here because it is a simple ADC-capable example pin on
// ESP32-S3 boards. If your board routes ADC differently, change this pin.
static constexpr uint8_t kPotPin = 4;
#else
static constexpr uint8_t kPotPin = A0;
#endif

auto pot = PotIO::makePotAnalog(kPotPin);

void setup()
{
    Serial.begin(115200);
}

void loop()
{
    pot.update();

    const auto s = pot.state();
    Serial.print("raw01=");
    Serial.print(s.raw01, 3);
    Serial.print(" centered=");
    Serial.println(s.centered, 3);

    delay(20);
}
```

---

## Core concepts

### Raw ADC input

`analogRead()` gives a board-specific raw ADC value. Depending on the board, this might be:

| Board family                   | Typical ADC range |
| ------------------------------ | ----------------: |
| Arduino Uno / AVR              |           0..1023 |
| ESP32 / ESP32-S3 Arduino       |           0..4095 |
| RP2040 / SAMD / STM32 / Teensy | depends on core and resolution settings |

PotIO does not call `analogReadResolution()` for you. If your board uses a different ADC resolution, define `POTIO_ARDUINO_ADC_FULLSCALE` before including `PotIO.h`, or provide a custom reader with its own `kFullScale`.

### `raw01`

`raw01` is the raw ADC reading mapped to:

```text
0.0 .. 1.0
```

### `centered`

`centered` maps the control around the middle:

```text
-1.0 .. +1.0
```

This is useful for steering-style inputs, joysticks, trim controls, and anything with a meaningful center.

### Calibration

A real pot often does not reach perfect ADC endpoints. Instead of assuming `0`, `2048`, and `4095`, measure your actual hardware.

```cpp
PotIO::PotCalib calib{180, 2040, 3890};
auto pot = PotIO::makePotAnalog(kPotPin, calib);
```

PotIO maps calibration like this:

```text
min    -> 0.0
center -> 0.5
max    -> 1.0
```

Then `centered` becomes:

```text
min    -> -1.0
center ->  0.0
max    -> +1.0
```

If calibration is invalid or incomplete, PotIO falls back to simple full-scale mapping. That keeps sketches usable while you are still measuring real hardware.

---

## Devices

### LinearPot

Use `LinearPot` for a normal one-axis analog control.

Typical uses:

- throttle pedal
- slider
- trim knob
- steering feedback pot
- any analog input where one value matters

Main outputs:

- `state().raw01`
- `state().calib01`
- `state().centered`
- `frame()`

### Joystick2D

Use `Joystick2D` for two analog axes.

Typical uses:

- KY-023 joystick
- thumb stick
- two-axis input panel

Main outputs:

- `x()` / `state().x`
- `y()` / `state().y`
- `magnitude()` / `state().mag`
- `angleRad()` / `angleDeg()`
- `frame()`

Deadzone options:

| Deadzone       | Meaning                                              |
| -------------- | ---------------------------------------------------- |
| `None`         | No deadzone                                          |
| `Axial`        | Deadzone per axis                                    |
| `Radial`       | Circular joystick deadzone                           |
| `RadialScaled` | Circular deadzone, then rescale outside the deadzone |

For joysticks, `RadialScaled` is usually the most natural beginner default.

### ContinuousPot

Use `ContinuousPot` when you want a knob-like input with wrap tracking.

It tracks:

- current phase `0.0 .. 1.0`
- signed turns
- unwrapped angle in degrees/radians

Important: a normal potentiometer is not truly endless. PotIO can track wrap-style movement from a cyclic analog signal, but the physical hardware still matters.

### SteppedPot

Use `SteppedPot` when you want an analog input to behave like a mode selector.

```cpp
auto modeKnob = PotIO::makeSteppedPot<5>(kPotPin);
```

This gives stable steps:

```text
0, 1, 2, 3, 4
```

The hysteresis setting helps prevent flickering between neighboring steps.

---

## Filters, shaping, and rate limiting

PotIO separates the analog input from the feel of the control.

### Filters

| Filter      | Use                                   |
| ----------- | ------------------------------------- |
| `NoFilter`  | Raw response, no smoothing            |
| `EMAFilter` | Simple smoothing for noisy ADC values |

```cpp
PotIO::EMAFilter f;
f.alpha = 0.12f;
```

Lower alpha is smoother but slower. Higher alpha follows faster but lets more noise through.

### Rate limiting

| Rate limiter  | Use                                |
| ------------- | ---------------------------------- |
| `NoRateLimit` | No output speed limit              |
| `SlewRate`    | Limit how fast the output can move |

```cpp
PotIO::SlewRate r;
r.units_per_s = 3.0f;
```

This is useful when a noisy or jumpy input should not instantly jump the interpreted output.

### Shaping

| Shaper           | Use                                      |
| ---------------- | ---------------------------------------- |
| `ShapeIdentity`  | Linear response                          |
| `ShapeCubicExpo` | Gentler near center, stronger near edges |
| `ShapeSoftZone`  | Softer center with progressive edges     |

Shaping changes the feel of a control. It should not be used as a substitute for real calibration.

---

## ESP32 ADC notes

ESP32 ADC behavior is more complicated than classic Arduino ADC behavior.

For ESP32 / ESP32-S3 projects:

- Prefer ADC1-capable pins for analog controls.
- Choose attenuation deliberately.
- Calibrate using real observed readings.
- Do not assume raw ADC values are perfectly linear.
- Do not feed 5 V into an ESP32 ADC pin.

Example ESP32-specific setup:

```cpp
#if defined(ARDUINO_ARCH_ESP32)
PotIO::setEsp32PinAttenuation(kPotPin, 11);
#endif
```

This belongs in ESP32-specific examples. The beginner examples intentionally keep these details out of the main flow.

---

## SnapshotBus integration

PotIO devices are intended to be updated from one context. If another task or core needs the latest value, publish the `frame()` to SnapshotBus.

Conceptual pattern:

```cpp
pot.update();
auto f = pot.frame();
// bus.publish(f);
```

This keeps the layers clean:

- PotIO reads and shapes analog input.
- SnapshotBus moves the latest state across tasks/cores.
- The application decides what to do with the value.

---

## Examples

The library examples are numbered from beginner to more advanced:

| Example                     | Purpose                                                     |
| --------------------------- | ----------------------------------------------------------- |
| `01_LinearPot_Basic`        | Read one potentiometer and print normalized values          |
| `02_Pot_Calibration`        | Measure real min/center/max values                          |
| `03_ContinuousPot_Advanced` | Use a continuous knob with wrap tracking                    |
| `04_Joystick2D_KY023`       | Read a two-axis analog joystick                             |
| `05_ESP32_ADC`              | ESP32-specific ADC attenuation and millivolt reader example |
| `06_Pot_Shaping`            | Compare response shaping styles                             |
| `07_Pot_Steering`           | Use a pot as steering-style input, without motor control    |
| `08_SteppedPot_ModeKnob`    | Convert analog input into stable discrete mode steps        |

General examples use `Serial.print()` / `Serial.println()` rather than `Serial.printf()` so they remain friendly to smaller Arduino cores.

---

## Common mistakes

### Treating raw ADC values as calibrated position

Raw ADC counts are not the same as real physical position. Measure min, center, and max where it matters.

### Forgetting ADC voltage limits

Do not feed a 5 V signal into a 3.3 V ADC pin.

### Assuming every board uses `0..4095`

Many Arduino boards use `0..1023` by default. Check your board and configure full scale if needed.

### Copying old ESP32 ADC pins onto ESP32-S3

Older ESP32 sketches often use GPIO34/GPIO35. For ESP32-S3 projects, use ADC-capable pins for that chipset. The examples use GPIO4/GPIO5 for ESP32 builds.

### Assuming every ESP8266 board can read two analog axes

Many ESP8266 boards expose only one ADC pin. The two-axis examples compile on ESP8266 for portability, but real joystick input usually needs an external ADC or a board with two analog inputs.

### Using ESP32 ADC2 pins with Wi-Fi without checking the board/core behavior

ESP32 ADC behavior varies by chip family and Arduino core. If Wi-Fi is involved, prefer known-good ADC1 pins and test your exact board.

### Adding motor control into the input layer

PotIO should not drive motors. Keep input handling separate from control and output layers.

---

## Project structure

```text
PotIO/
  src/                         Library headers
  src/devices/                 LinearPot, ContinuousPot, Joystick2D, SteppedPot
  src/devices/helpers/         Small convenience facades and presets
  examples/                    Numbered Arduino examples
  library.json                 PlatformIO library metadata
  library.properties           Arduino library metadata
  platformio.ini               Local development config
  platformio.ci.ini            Example compile-validation config
```

The release package should not include PlatformIO build output, VS Code machine files, macOS metadata, generated ZIP files, or a sketch-style `src/main.cpp`.

---

## Version note

PotIO is currently unreleased at version `1.0.0`. Release-prep cleanup should not change the version until a real release tag is being made.

---

## License

MIT © Little Man Builds
