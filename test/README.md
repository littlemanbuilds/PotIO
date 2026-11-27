# PotIO tests

The test tree is intentionally part of the library release.

- `run_native_tests.sh` builds deterministic device and policy contract tests under C++11 with strict compiler warnings.
- `run_sanitizers.sh` runs the same native contracts under AddressSanitizer and UndefinedBehaviorSanitizer.
- `check_examples_syntax.sh` compiles every public ESP32-S3-oriented example and the portable smoke sketch. It also runs the calibration example with valid and reversed ADC readings.
- `check_release_contracts.sh` validates version/manifest consistency, documentation structure, source documentation headers, package hygiene, and core validity contracts.
- `run_host_checks.sh` runs the complete host-side validation suite above.
- `portable_compile/` is the small public-API sketch used by the PlatformIO board matrix.

Host tests deliberately avoid an external framework so the core contracts can run in a minimal CI environment. Hardware ADC behavior, attenuation, source impedance, electrical noise, and true cross-platform toolchain compilation remain separate validation gates.

## Release archives

`package_inventory.txt` lists the complete public Git tree. The Arduino Library
Manager reference uses its non-dotfile projection. PlatformIO uses the distribution
list in `library.json`, which includes source, public examples, metadata, docs, and
the license.

From the protected LMB coordinator, run the following with the installed v4.1.1
Harness and the Python interpreter for PlatformIO Core 6.1.19. Supply the public
[SPDX 3.27.0 license catalog](https://raw.githubusercontent.com/spdx/license-list-data/v3.27.0/json/licenses.json)
as an explicit downloaded input for offline manifest validation. The packager
verifies its SHA-256. The release tag must already resolve to the exact release
commit.

```sh
python3 -B test/package_release.py --commit <full-commit-sha> \
  --harness <harness-directory> --pio-python <pio-core-python> \
  --spdx <catalog-json> <new-private-output-directory>
```

The script prints the distribution plan for the coordinator's archive check.
`--preview` checks a working-copy PlatformIO package for local testing.
Run `test/test_release_packaging.py` through the protected runner to exercise the
public inventory, Git entry checks, Core export rules, and corrupt archive cases.
