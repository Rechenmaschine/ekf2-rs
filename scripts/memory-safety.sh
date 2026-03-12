#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

echo "[memory-safety] Running low-level FFI boundary guard tests..."
cargo test -p ekf2-sys --test memory_interface

echo "[memory-safety] Running Rust-side storage and lifecycle checks..."
cargo test -p ekf2 --lib

echo "[memory-safety] Running Miri checks for Rust-only storage invariants..."
cargo miri test -p ekf2 --lib storage::tests::aligned_buffer_has_expected_alignment -- --exact
cargo miri test -p ekf2 --lib storage::tests::pool_ptr_points_to_pool_storage -- --exact
cargo miri test -p ekf2 --lib storage::tests::ekf_ptr_points_to_object_storage_and_is_aligned -- --exact
cargo miri test -p ekf2 --lib storage::tests::uninit_can_be_used_in_const_context -- --exact

echo "[memory-safety] Done."
