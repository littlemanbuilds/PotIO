#!/usr/bin/env sh
set -eu
ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
exec python3 -B "$ROOT_DIR/test/check_release_contracts.py" "$@"
