# PotIO tests

The test tree is intentionally part of the library release.

- `run_native_tests.sh` builds deterministic device and policy contract tests under C++11 with strict compiler warnings.
- `run_sanitizers.sh` runs the same native contracts under AddressSanitizer and UndefinedBehaviorSanitizer.
- `check_examples_syntax.sh` compiles every public ESP32-S3-oriented example against the real headers and a small Arduino/ESP32 stand-in, then compiles the portable smoke sketch.
- `check_release_contracts.sh` validates version/manifest consistency, documentation structure, LMB Doxygen headers, package hygiene, and the presence of the v1.1 safety contracts.
- `run_host_checks.sh` runs the complete host-side validation suite above.
- `portable_compile/` is the small public-API sketch used by the PlatformIO board matrix.

Host tests deliberately avoid an external framework so the core contracts can run in a minimal CI environment. Hardware ADC behavior, attenuation, source impedance, electrical noise, and true cross-platform toolchain compilation remain separate validation gates.
