#!/usr/bin/env sh
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)

# On systems where g++ is a Clang alias, select GNU C++ explicitly with GCC_CXX.
GCC_CXX=${GCC_CXX:-g++}
CLANG_CXX=${CLANG_CXX:-clang++}
if ! "$GCC_CXX" --version | grep -q 'Free Software Foundation'; then
    printf 'GCC_CXX must select a GNU C++ compiler.\n' >&2
    exit 1
fi
if ! "$CLANG_CXX" --version | grep -qi clang; then
    printf 'CLANG_CXX must select a Clang C++ compiler.\n' >&2
    exit 1
fi
CXX="$GCC_CXX" sh "$ROOT_DIR/test/run_native_tests.sh"
CXX="$CLANG_CXX" sh "$ROOT_DIR/test/run_native_tests.sh"
CXX="$CLANG_CXX" sh "$ROOT_DIR/test/run_sanitizers.sh"
CXX="$CLANG_CXX" sh "$ROOT_DIR/test/check_examples_syntax.sh"
sh "$ROOT_DIR/test/check_release_contracts.sh"

printf 'All PotIO host validation checks passed.\n'
