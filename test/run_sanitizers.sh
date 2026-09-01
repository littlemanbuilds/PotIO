#!/usr/bin/env sh
set -eu
ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
BUILD_DIR=$(mktemp -d "${TMPDIR:-/tmp}/potio-sanitize.XXXXXX")
trap 'rm -rf "$BUILD_DIR"' EXIT INT TERM
CXX=${CXX:-c++}
FLAGS="-std=c++11 -Wall -Wextra -Werror -pedantic -Wconversion -Wsign-conversion -Wshadow -fno-omit-frame-pointer -fsanitize=address,undefined"
# shellcheck disable=SC2086
"$CXX" $FLAGS -I"$ROOT_DIR/src" "$ROOT_DIR/test/native/test_main.cpp" -o "$BUILD_DIR/potio_sanitize"
if [ "$(uname -s)" = "Darwin" ]; then
    ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=print_stacktrace=1 "$BUILD_DIR/potio_sanitize"
else
    ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=print_stacktrace=1 "$BUILD_DIR/potio_sanitize"
fi
printf 'ASan/UBSan checks passed.\n'
