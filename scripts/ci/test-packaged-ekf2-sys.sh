#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="$(mktemp -d)"
trap 'rm -rf "$BUILD_DIR"' EXIT

PACKAGE_TARGET="$BUILD_DIR/package-target"
TEST_TARGET="$BUILD_DIR/test-target"

cd "$ROOT_DIR"

cargo package \
  -p ekf2-sys \
  --no-verify \
  --locked \
  --allow-dirty \
  --target-dir "$PACKAGE_TARGET"

CRATE="$(find "$PACKAGE_TARGET/package" -maxdepth 1 -type f -name 'ekf2-sys-*.crate' -print -quit)"
if [ -z "$CRATE" ]; then
  echo "FAIL: packaged ekf2-sys crate not found" >&2
  exit 1
fi

tar -xf "$CRATE" -C "$BUILD_DIR"
PACKAGE_DIR="$(find "$BUILD_DIR" -maxdepth 1 -type d -name 'ekf2-sys-*' -print -quit)"

cargo test \
  --manifest-path "$PACKAGE_DIR/Cargo.toml" \
  --all-features \
  --locked \
  --target-dir "$TEST_TARGET"
