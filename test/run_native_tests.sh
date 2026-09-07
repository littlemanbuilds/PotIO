#!/usr/bin/env sh
set -eu
ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR=$(mktemp -d "${TMPDIR:-/tmp}/potio-native.XXXXXX")
trap 'rm -rf "$BUILD_DIR"' EXIT INT TERM
CXX=${CXX:-c++}
FLAGS="-std=c++11 -Wall -Wextra -Werror -pedantic -Wconversion -Wsign-conversion -Wshadow"
"$CXX" $FLAGS -I"$ROOT_DIR/src" "$ROOT_DIR/test/native/test_main.cpp" -o "$BUILD_DIR/potio_tests"
"$BUILD_DIR/potio_tests"

# Separate translation units exercise the documented fallback macro without
# changing the normal suite's calibration and reader assumptions.
for fullscale in 1023 65535 0 -1 65536 4294968319ULL; do
    "$CXX" $FLAGS -DPOTIO_DEFAULT_FULLSCALE="$fullscale" -I"$ROOT_DIR/src" \
        "$ROOT_DIR/test/native/test_fallback_fullscale.cpp" -o "$BUILD_DIR/fallback_tests"
    "$BUILD_DIR/fallback_tests"
done
