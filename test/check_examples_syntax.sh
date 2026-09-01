#!/usr/bin/env sh
set -eu
ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR=$(mktemp -d "${TMPDIR:-/tmp}/potio-examples.XXXXXX")
trap 'rm -rf "$BUILD_DIR"' EXIT INT TERM
CXX=${CXX:-c++}
FLAGS="-std=c++11 -Wall -Wextra -Werror -pedantic -Wconversion -Wsign-conversion -Wshadow"
INCLUDES="-I$ROOT_DIR/test/support -I$ROOT_DIR/src"
DEFINES="-DARDUINO=10819 -DARDUINO_ARCH_ESP32 -DESP32"
# shellcheck disable=SC2086
"$CXX" $FLAGS $INCLUDES $DEFINES -c "$ROOT_DIR/test/support/ExampleRuntime.cpp" -o "$BUILD_DIR/runtime.o"
count=0
for example in "$ROOT_DIR"/examples/*/*.ino; do
    name=$(basename "$example" .ino)
    # shellcheck disable=SC2086
    "$CXX" $FLAGS $INCLUDES $DEFINES -x c++ -c "$example" -o "$BUILD_DIR/$name.o"
    count=$((count + 1))
done
# shellcheck disable=SC2086
"$CXX" $FLAGS $INCLUDES $DEFINES -x c++ -c "$ROOT_DIR/test/portable_compile/portable_compile.ino" -o "$BUILD_DIR/portable.o"
printf 'Example syntax checks passed: %s ESP32-S3-oriented public examples + portable smoke sketch.\n' "$count"
