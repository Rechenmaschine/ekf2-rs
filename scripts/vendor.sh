#!/usr/bin/env bash
# vendor.sh — update the pinned PX4-Autopilot submodule commit.
#
# Usage:
#   ./scripts/vendor.sh            # pin to latest origin/main
#   ./scripts/vendor.sh <commit>   # pin to a specific PX4 commit hash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$ROOT_DIR"

SUBMODULE_PATH="ekf2-sys/vendor/PX4-Autopilot"
REQUESTED_COMMIT="${1:-}"

if [[ ! -f .gitmodules ]]; then
    echo "[vendor.sh] ERROR: .gitmodules not found."
    echo "[vendor.sh] Ensure the repository includes the PX4 submodule."
    exit 1
fi

echo "[vendor.sh] Initializing/updating submodule..."
git submodule update --init --recursive "$SUBMODULE_PATH"

cd "$SUBMODULE_PATH"

if [[ -n "$REQUESTED_COMMIT" ]]; then
    echo "[vendor.sh] Pinning to requested commit: $REQUESTED_COMMIT"
    git fetch origin "$REQUESTED_COMMIT" || git fetch origin
    git checkout --detach "$REQUESTED_COMMIT"
else
    echo "[vendor.sh] Pinning to latest origin/main"
    git fetch origin main
    git checkout --detach origin/main
fi

ACTUAL_COMMIT="$(git rev-parse HEAD)"
cd "$ROOT_DIR"

git add "$SUBMODULE_PATH"

echo ""
echo "[vendor.sh] Submodule pinned:"
echo "  $SUBMODULE_PATH @ $ACTUAL_COMMIT"
echo ""
echo "[vendor.sh] Next step: commit the updated submodule pointer."
