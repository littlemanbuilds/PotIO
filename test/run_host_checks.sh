#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)

sh "$ROOT_DIR/test/run_native_tests.sh"
sh "$ROOT_DIR/test/run_sanitizers.sh"
sh "$ROOT_DIR/test/check_examples_syntax.sh"
sh "$ROOT_DIR/test/check_release_contracts.sh"

printf 'All PotIO host validation checks passed.\n'
