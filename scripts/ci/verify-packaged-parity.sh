#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(
  cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd
)
cd "$ROOT_DIR"

BUILD=$(mktemp -d)
trap 'rm -rf "$BUILD"' EXIT

PKG_SYS_TAR_TARGET="$BUILD/package-target"
GIT_SYS="$BUILD/ekf2-sys-git"
PKG_WORKSPACE="$BUILD/packaged-workspace"
PKG_SYS_TARGET="$BUILD/target-pkg-sys"
GIT_SYS_TARGET="$BUILD/target-git-sys"
PKG_WS_TARGET="$BUILD/target-pkg-workspace"

cpp_symbols() {
  local target_dir="$1"
  local archive

  archive=$(find "$target_dir" -path '*/out/libekf2_cpp.a' -print -quit)
  [ -n "$archive" ] || {
    echo "FAIL: libekf2_cpp.a not found under $target_dir" >&2
    exit 1
  }

  nm "$archive" | sort | sha256sum | awk '{print $1}'
}

copy_packaged_files() {
  local pkg="$1"
  local src_root="$2"
  local dest_root="$3"

  mkdir -p "$dest_root"

  cargo package -p "$pkg" --list --locked --allow-dirty | while IFS= read -r rel; do
    case "$rel" in
      ''|.cargo_vcs_info.json|Cargo.lock|Cargo.toml.orig)
        continue
        ;;
    esac

    local from="$src_root/$rel"
    if [ ! -e "$from" ]; then
      from="$ROOT_DIR/$rel"
    fi

    [ -e "$from" ] || {
      echo "FAIL: missing packaged file for $pkg: $rel" >&2
      exit 1
    }

    mkdir -p "$dest_root/$(dirname "$rel")"
    cp "$from" "$dest_root/$rel"
  done
}

# ekf2-sys can be tested from a real packaged tarball.
cargo package -p ekf2-sys --no-verify --locked --allow-dirty --target-dir "$PKG_SYS_TAR_TARGET"
tar xf "$PKG_SYS_TAR_TARGET"/package/ekf2-sys-*.crate -C "$BUILD"
PKG_SYS=$(ls -d "$BUILD"/ekf2-sys-*)

cargo test --manifest-path "$PKG_SYS/Cargo.toml" --all-features --locked --target-dir "$PKG_SYS_TARGET"
SYM_PKG=$(cpp_symbols "$PKG_SYS_TARGET")

# Compare against the git tree while reusing the packaged manifest and lockfile.
cp -R ekf2-sys "$GIT_SYS"
cp "$PKG_SYS/Cargo.toml" "$GIT_SYS/Cargo.toml"
cp "$PKG_SYS/Cargo.lock" "$GIT_SYS/Cargo.lock"
cargo build --manifest-path "$GIT_SYS/Cargo.toml" --lib --all-features --locked --target-dir "$GIT_SYS_TARGET"
SYM_GIT=$(cpp_symbols "$GIT_SYS_TARGET")

# ekf2 cannot be tested from a real .crate until ekf2-sys is published, so stage a
# temporary workspace from the exact source-backed packaged file sets instead.
mkdir -p "$PKG_WORKSPACE"
cp Cargo.toml "$PKG_WORKSPACE/Cargo.toml"
cp Cargo.lock "$PKG_WORKSPACE/Cargo.lock"
copy_packaged_files ekf2-sys ekf2-sys "$PKG_WORKSPACE/ekf2-sys"
copy_packaged_files ekf2 ekf2 "$PKG_WORKSPACE/ekf2"
cargo test \
  --manifest-path "$PKG_WORKSPACE/Cargo.toml" \
  --workspace \
  --all-features \
  --locked \
  --target-dir "$PKG_WS_TARGET"

echo "Package symbols: $SYM_PKG"
echo "Git symbols:     $SYM_GIT"
[ "$SYM_PKG" = "$SYM_GIT" ] \
  && echo "PASS: ekf2-sys packaged and git C++ symbol tables match" \
  || {
    echo "FAIL: symbol tables differ — packaged ekf2-sys is missing or has wrong C++ sources" >&2
    exit 1
  }
