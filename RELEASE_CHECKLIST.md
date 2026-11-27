# PotIO Release Checklist

Use this checklist for PotIO v1.2.0 before creating a release archive or registry submission.

## Source and API

- [ ] `POTIO_VERSION`, `library.properties`, and `library.json` match.
- [ ] Only `POTIO_VERSION*` package-specific version macros are exposed; generic `LIBRARY_VERSION*` aliases remain absent.
- [ ] Public API remains C++11 compatible.
- [ ] Public source files retain complete Doxygen documentation headers.
- [ ] No accidental dependency on another LMB library is introduced.
- [ ] Reader failure retains last-good output and reports invalid status.
- [ ] Calibration/timing/configuration errors cannot become plausible fresh commands.
- [ ] Disabled raw bounds preserve endpoint behaviour; enabled bounds reject implausible input before processing.
- [ ] Rejection retains sample identity, joystick atomicity, continuous tracking, and stepped event history.
- [ ] Both constructors reject non-positive and oversized reader full-scale declarations.

## Tests

- [ ] `./test/run_native_tests.sh` passes with strict warnings under both GCC and Clang.
- [ ] `./test/run_sanitizers.sh` passes under ASan/UBSan.
- [ ] `./test/check_examples_syntax.sh` passes.
- [ ] `./test/check_release_contracts.sh` passes.
- [ ] `./test/run_host_checks.sh` passes from a clean checkout without generated files.
- [ ] GitHub Actions compile matrix passes on the release commit.
- [ ] ESP32-S3 examples compile against the pinned Arduino-ESP32 environment in `platformio.ini`.

## Hardware

Physical analogue fault validation is deferred to project/hardware integration;
compilation and native tests do not complete these checks.

- [ ] Plausibility bounds established from measured travel and fault behaviour with margin.
- [ ] Rail/open/short and noise behaviour checked on the actual circuit.

- [ ] Basic raw ADC reading checked on an ESP32-S3 DevKitC-1.
- [ ] Calibration example exercised on real hardware.
- [ ] Joystick center/deadzone behavior checked with a real two-axis stick.
- [ ] ESP32 attenuation/millivolt example checked on real hardware if changed.
- [ ] Any ContinuousPot tuning change checked with the intended cyclic hardware.

## Documentation and package

- [ ] README beginner path matches the current API.
- [ ] README technical reference matches defaults and failure semantics.
- [ ] All public examples remain beginner-friendly.
- [ ] `keywords.txt` includes new public types/methods.
- [ ] `CHANGELOG.md` records every user-visible change.
- [ ] The committed tree and Git index match `test/package_inventory.txt`.
- [ ] GitHub source ZIP/TAR, PlatformIO TAR, and Arduino ZIP references pass the exact-commit distribution check.
- [ ] The unpacked source ZIP passes the host checks, and the unpacked PlatformIO package compiles on ESP32-S3.
- [ ] Every final archive's SHA-256 is bound to the release review.
- [ ] The actual public downloads pass the per-channel inventory, byte, and archive checks.
