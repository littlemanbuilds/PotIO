# Changelog

All notable PotIO changes are documented here.

## [1.2.0] - 2026-11-27

### Added

- Optional inclusive `RawBounds` on all four devices, with independent joystick axis bounds, disabled by default.
- Appended `ReadError::ImplausibleSample` for successful in-range acquisitions outside the configured credible envelope. Rejection precedes mapping and processing, retains last-good output and sample identity, and follows existing recovery policies.
- Configuration validation for ordered, representable bounds within reader range and surrounding otherwise valid calibrated travel.
- Focused native regressions for raw rejection, processing history, joystick atomicity, continuous tracking, stepped events, and reader representation limits.
- Raw plausibility guidance and an illustrative steering configuration; physical fault validation remains deferred to hardware integration.

### Fixed

- Default-constructed devices now cache custom-reader full-scale consistently with Config-constructed devices, including rejection of non-positive declarations.
- Reader full-scale declarations above `UINT16_MAX`, including the configured fallback for readers without `kFullScale`, are rejected before narrowing to prevent truncation of public raw/calibration coordinates.
- PlatformIO exports are limited to the library source, examples, metadata, documentation, and license. Release checks compare each channel's archive with its intended inventory and committed file contents.
- The calibration example rejects invalid captures before printing calibration settings to copy.

### Changed

- PlatformIO CI uses Python 3.11 and PlatformIO Core 6.1.19.

## [1.1.0] - 2026-09-01

### Fixed

- Removed generic `LIBRARY_VERSION*` aliases from the public surface because those global preprocessor names collide across libraries; `POTIO_VERSION*` remains authoritative.
- Negative/error reader results can no longer be clamped into a valid zero/full-negative control command.
- Reader values above the declared full-scale are rejected instead of silently clamped.
- LinearPot, Joystick2D, and ContinuousPot now keep filter history separate from final slew-limited output history.
- Invalid, non-finite, negative, or excessive update timing no longer enters the slew/rate pipeline.
- Invalid enum values supplied through casts are rejected as invalid configuration instead of falling through to an unintended policy.
- Radial-scaled joystick deadzones no longer amplify vectors that are already outside the unit circle.
- Neutral joystick angle is no longer presented as semantically valid.
- ContinuousPot can detect physically ambiguous/skipped movement rather than silently corrupting turn tracking.
- ContinuousPot plausibility uses the last successful phase timestamp, so failed acquisitions cannot hide an alias-prone sampling gap.
- SteppedPot change information can survive latest-state transport via a monotonic change sequence.
- JitterStats no longer lets one lifetime outlier dominate current noise measurement indefinitely.
- Non-finite values returned by custom shaping, filtering, or rate-limit policies are rejected instead of being published as valid output.
- ESP32 ADC attenuation now uses the Arduino core enum names accepted by the real ESP32-S3 PlatformIO toolchain.
- Beginner examples now report invalid samples before printing retained last-good values.

### Added

- `RawSample` for explicit reader success/failure.
- `SampleStatus` with validity, last-good sample time, successful sample sequence, calibration validity, error, and quality flags.
- `ReadError` and `QualityFlag` contracts.
- `CalibrationPolicy::RequireValid` and permissive fallback reporting.
- `InvalidSamplePolicy` for filter/rate recovery history.
- Calibration validation against reader full-scale and minimum center spans.
- `Deadzone::AxialScaled`.
- `JoystickGeometry::{Square, MagnitudeClamp, SquareToCircle}`.
- `Joystick2D::angleValid()` / `State::angle_valid`.
- ContinuousPot `max_phase_delta`, `max_turns_per_s`, `turns_valid`, and `resynchronizeTurns()`.
- SteppedPot `change_sequence` / `changeSequence()`.
- `RollingJitterStats<Window>` with `JitterStats` as the 32-sample default.
- Dedicated native tests, example syntax checks, portable compile smoke test, release-contract checks, and GitHub Actions compile matrix.
- `test/run_host_checks.sh` as one local command for the complete host-side validation suite.
- Standardized README, package, and source-documentation structure and a dedicated beginner's guide.

### Changed

- Version advanced from v1.0.0 to v1.1.0 as a backward-compatible minor release with new public capabilities and intentional defect corrections.
- Runtime calibration changes re-seed affected processing state rather than mixing old and new coordinate systems.
- Uncalibrated ESP32 millivolt factories now seed a calibration matching `FullScaleMv`.
- `JitterStats` now measures a recent rolling window rather than lifetime min/max.
- Legacy `platformio.ci.ini` replaced by the current test and CI configuration.

## [1.0.0] - 2026-06-02

- Initial PotIO release with LinearPot, Joystick2D, ContinuousPot, SteppedPot, filters, shaping, rate limiting, Arduino readers, factory helpers, and examples.
