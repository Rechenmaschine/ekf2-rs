#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[miri] Running low-level FFI boundary guard tests (native)..."
cargo test -p ekf2-sys --test memory_interface

echo "[miri] Running Miri checks..."
cargo miri test -p ekf2 --lib --no-default-features
cargo miri test -p ekf2-sys --lib --no-default-features

echo "[miri] Done."
