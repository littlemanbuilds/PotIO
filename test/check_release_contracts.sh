#!/usr/bin/env sh
set -eu
ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
python3 - "$ROOT_DIR" <<'PY'
import json
import re
import sys
from pathlib import Path

root = Path(sys.argv[1])
errors = []

def require(condition, message):
    if not condition:
        errors.append(message)

required = [
    '.gitignore', 'CHANGELOG.md', 'LICENSE', 'README.md', 'RELEASE_CHECKLIST.md',
    'keywords.txt', 'library.json', 'library.properties', 'platformio.ini',
    '.github/workflows/ci.yml', 'test/README.md', 'test/run_native_tests.sh',
    'test/run_sanitizers.sh', 'test/run_host_checks.sh', 'test/check_examples_syntax.sh',
    'test/check_release_contracts.sh', 'test/portable_compile/portable_compile.ino'
]
for rel in required:
    require((root / rel).is_file(), 'missing required file: ' + rel)

props = {}
for line in (root / 'library.properties').read_text().splitlines():
    if '=' in line:
        key, value = line.split('=', 1)
        props[key.strip()] = value.strip()
manifest = json.loads((root / 'library.json').read_text())
compat = (root / 'src/PotIO_Compatibility.h').read_text()
version_match = re.search(r'#define\s+POTIO_VERSION\s+"([^"]+)"', compat)
version = version_match.group(1) if version_match else ''
require(version == '1.1.0', 'POTIO_VERSION must be 1.1.0')
for macro, expected in [
    ('POTIO_VERSION_MAJOR', '1'),
    ('POTIO_VERSION_MINOR', '1'),
    ('POTIO_VERSION_PATCH', '0'),
]:
    require(re.search(r'#define\s+' + macro + r'\s+' + re.escape(expected) + r'\b', compat),
            macro + ' mismatch')
require(props.get('version') == version, 'library.properties version mismatch')
require(manifest.get('version') == version, 'library.json version mismatch')
require(manifest.get('license') == 'MIT', 'library.json license must be MIT')
require(props.get('license') == 'MIT', 'library.properties license must be MIT')
require(manifest.get('headers') == ['PotIO.h'], 'library.json public header manifest changed unexpectedly')
require(not re.search(r'^#define\s+LIBRARY_VERSION(?:_|\s)', compat, flags=re.M), 'generic LIBRARY_VERSION aliases must remain absent')

ignored = []
for line in (root / '.gitignore').read_text().splitlines():
    stripped = line.strip()
    if stripped and not stripped.startswith('#'):
        ignored.append(stripped)
require('test/' not in ignored, '.gitignore must not ignore the test/ tree')
require('CHANGELOG.md' not in ignored, '.gitignore must not ignore CHANGELOG.md')
require('RELEASE_CHECKLIST.md' not in ignored, '.gitignore must not ignore RELEASE_CHECKLIST.md')

examples = sorted(str(p.relative_to(root)).replace('\\', '/') for p in root.glob('examples/*/*.ino'))
manifest_examples = sorted(manifest.get('examples', []))
require(len(examples) == 8, 'expected exactly eight public examples')
require(examples == manifest_examples, 'library.json example list does not match public examples')

readme = (root / 'README.md').read_text()
for heading in [
    '## Contents', '## Installation', "## Beginner's guide", '## The v1.1 validity model',
    '## Calibration', '## LinearPot', '## Joystick2D', '## ContinuousPot', '## SteppedPot',
    '## Testing and validation', '## Migration from v1.0.0', '## Deliberate limitations', '## License'
]:
    require(heading in readme, 'README missing section: ' + heading)
for token in ['RawSample', 'SampleStatus', 'CalibrationPolicy::RequireValid', 'AxialScaled',
              'JoystickGeometry', 'angleValid()', 'turnsValid()', 'resynchronizeTurns()',
              'change_sequence', 'RollingJitterStats']:
    require(token in readme, 'README missing v1.1 contract: ' + token)
required_tail = """## Repository structure

```text
"""
require(required_tail in readme, 'README repository structure must open with a text fence')
require(readme.endswith("""## License

PotIO is released under the **MIT License**. See [LICENSE](LICENSE).

Copyright © 2026 Little Man Builds (Darren Osborne).
"""), 'README final license tail does not match the public LMB contract')
require('development baseline' not in readme, 'README must describe the v1.1.0 release candidate, not a development baseline')
beginner = readme.split("## Beginner's guide", 1)[1]
require('```cpp\n#include <PotIO.h>\n#include <Arduino.h>\n' in beginner,
        'README beginner snippet must include PotIO.h before Arduino.h')

# Public LMB source/example header contract.
for path in sorted((root / 'src').rglob('*.h')) + sorted((root / 'examples').glob('*/*.ino')):
    text = path.read_text()
    rel = str(path.relative_to(root))
    require('MIT License' in text, rel + ': missing MIT header')
    require('@brief' in text, rel + ': missing @brief')
    require('@file' in text, rel + ': missing @file')
    require('@author Little Man Builds (Darren Osborne)' in text, rel + ': missing standardized author')
    require('@date 2026-06-02' in text, rel + ': creation date must remain 2026-06-02')
    require('@copyright Copyright © 2026 Little Man Builds' in text, rel + ': missing copyright')

# Concrete remediation markers: these intentionally target the audit findings.
types = (root / 'src/PotIO_Types.h').read_text()
detail = (root / 'src/PotIO_Detail.h').read_text()
linear = (root / 'src/devices/PotIO_LinearPot.h').read_text()
joystick = (root / 'src/devices/PotIO_Joystick2D.h').read_text()
continuous = (root / 'src/devices/PotIO_ContinuousPot.h').read_text()
stepped = (root / 'src/devices/PotIO_SteppedPot.h').read_text()
jitter = (root / 'src/PotIO_JitterTools.h').read_text()
for token in ['struct RawSample', 'struct SampleStatus', 'RequireValid', 'ResetProcessing', 'AxialScaled', 'enum class JoystickGeometry']:
    require(token in types, 'PotIO_Types.h missing: ' + token)
require('raw < 0' in detail and 'ReaderFailure' in detail, 'negative read failure contract missing')
require('filtered_centered_' in linear, 'LinearPot independent filter history missing')
require('angle_valid' in joystick and 'SquareToCircle' in joystick, 'Joystick v1.1 geometry/angle contract missing')
require('last_phase_ms_' in continuous and 'resynchronizeTurns' in continuous and 'Discontinuity' in continuous,
        'ContinuousPot plausibility/re-sync contract missing')
require('change_sequence' in stepped, 'SteppedPot durable change sequence missing')
require('class RollingJitterStats' in jitter, 'rolling jitter implementation missing')

# Legacy CI and tracked/generated package debris must not return.
require(not (root / 'platformio.ci.ini').exists(), 'legacy platformio.ci.ini must not be packaged')
require(not (root / 'prepare_potio_v1_1_0.sh').exists(), 'private pre-harness preparation script must not be packaged')
import subprocess
import tempfile

def inventory_paths(repo_root):
    top = subprocess.run(
        ['git', '-C', str(repo_root), 'rev-parse', '--show-toplevel'],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
        text=True,
    )
    if top.returncode != 0 or Path(top.stdout.strip()).resolve() != repo_root.resolve():
        return sorted(
            str(path.relative_to(repo_root)).replace('\\', '/')
            for path in repo_root.rglob('*')
            if path.is_file()
        )
    proc = subprocess.run(
        ['git', '-C', str(repo_root), 'ls-files', '-z'],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    if proc.returncode != 0:
        raise RuntimeError('could not read Git release inventory')
    return [
        item.decode('utf-8', errors='surrogateescape')
        for item in proc.stdout.split(b'\0')
        if item
    ]

def inventory_errors(paths):
    findings = []
    for relative in paths:
        path = Path(relative)
        if path.name == '.DS_Store' or path.name.startswith('._') or '__MACOSX' in path.parts:
            findings.append('unwanted release metadata: ' + relative)
        if relative == '.vscode/extensions.json':
            continue
        if any(part in {'.pio', '.vscode', 'build', 'dist', '__pycache__'} for part in path.parts):
            findings.append('unwanted generated release content: ' + relative)
        if path.suffix.lower() in {'.o', '.obj', '.elf', '.bin', '.hex', '.map', '.zip'}:
            findings.append('unwanted build/archive release file: ' + relative)
    return findings

errors.extend(inventory_errors(inventory_paths(root)))

with tempfile.TemporaryDirectory(prefix='potio-release-inventory-') as temp_dir:
    package = Path(temp_dir) / 'PotIO'
    package.mkdir()
    (package / 'rogue.zip').write_bytes(b'not a release asset')
    probe_errors = inventory_errors(inventory_paths(package))
    require(any('rogue.zip' in finding for finding in probe_errors),
            'non-Git release inventory must reject archive content')

if errors:
    for error in errors:
        print('ERROR:', error)
    sys.exit(1)
print('Release contracts passed: version, manifests, documentation, LMB headers, remediation markers, and package hygiene.')
PY
