# PotIO Release Checklist

Use this checklist before creating a PotIO release archive or registry submission.

## Source and API

- [ ] `POTIO_VERSION`, `library.properties`, and `library.json` match.
- [ ] Only `POTIO_VERSION*` package-specific version macros are exposed; generic `LIBRARY_VERSION*` aliases remain absent.
- [ ] Public API remains C++11 compatible.
- [ ] Public source files retain the LMB Doxygen header style.
- [ ] No accidental dependency on another LMB library is introduced.
- [ ] Reader failure retains last-good output and reports invalid status.
- [ ] Calibration/timing/configuration errors cannot become plausible fresh commands.

## Tests

- [ ] `./test/run_native_tests.sh` passes with strict warnings.
- [ ] `./test/run_sanitizers.sh` passes under ASan/UBSan.
- [ ] `./test/check_examples_syntax.sh` passes.
- [ ] `./test/check_release_contracts.sh` passes.
- [ ] `./test/run_host_checks.sh` passes from a clean checkout without generated files.
- [ ] GitHub Actions compile matrix passes on the release commit.
- [ ] ESP32-S3 examples compile against the current Arduino-ESP32 core.

## Hardware

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
- [ ] No `.DS_Store`, `__MACOSX`, build output, archives, or editor state is packaged.
- [ ] Release ZIP is unpacked and all local tests are rerun from the unpacked copy.
- [ ] Final archive checksum is recorded.
