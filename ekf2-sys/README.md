# ekf2-sys

Raw `unsafe` FFI bindings for the PX4 EKF2 C++ navigation filter.

Most users should use the safe [`ekf2`](../ekf2) crate instead. This crate is
the low-level foundation that `ekf2` builds on.

## Crate contents

| Item | Description |
|---|---|
| Generated bindings | All types and `extern "C"` functions from `ekf2_wrapper.h`, re-exported via `pub use generated::*` |
| Allocator shims | `ekf2_rust_alloc`, `ekf2_rust_alloc_zeroed`, `ekf2_rust_dealloc` — C ABI symbols that route C++ heap calls through Rust's global allocator |
| Helper types | `HeightSensor`, `GlobalOrigin`, `PositionFrame`, `VelocityFrame` |
| Sample constructors | `impl` blocks on generated sensor structs (`EkfImuSample::new`, `EkfGnssSample::new`, …) |

## Architecture: the C++ shim layer

The PX4 EKF2 algorithm is a C++ class (`Ekf`) with no stable C ABI. This crate
wraps it in three files under `wrapper/`:

### `ekf2_wrapper.h` — the C API surface

Defines a pure-C `extern "C"` interface that bindgen can consume. All PX4 C++
types are mirrored as plain C structs with explicit padding (`_pad` fields).
Struct layouts are verified at compile time with `static_assert` in the `.cpp`
translation unit so that any upstream layout drift is caught immediately.

The lifetime contract exposed through this header has two modes:

- **Heap-owned** (used by the safe `ekf2` crate):
  `ekf2_create_heap()` → `ekf2_init()` → ... → `ekf2_destroy_heap()`
- **Caller-buffer** (for low-level / bare-metal use without a heap):
  `ekf2_create(buf, sizeof_buf)` placement-news the `Ekf` object into a
  caller-supplied buffer → `ekf2_init()` → ... → `ekf2_destroy()` (destructor
  only, no free)

### `ekf2_wrapper.cpp` — the C shims

Each `extern "C"` function in the header is a thin wrapper around the
corresponding `Ekf` method. Sensor sample structs are converted from the C
representation to the internal C++ representation via field-by-field copy. No
dynamic dispatch or virtual calls are involved; the compiler can inline freely.

Minimal stub `uORB/topics/estimator_aid_source{1,2,3}d.h` headers are bundled
under `wrapper/uORB/`. These satisfy include paths hardcoded inside PX4's
`ekf.h` without pulling in the uORB pub/sub machinery.

### `allocator.cpp` — the allocator bridge

C++ provides no portable way to swap the global allocator at link time, so this
file overrides **all** `operator new` / `operator delete` variants and, on
bare-metal targets, the `malloc` / `free` / `calloc` / `realloc` libc symbols.
Every allocation is forwarded to three C symbols:

```
ekf2_rust_alloc(size, align)        → u8*
ekf2_rust_alloc_zeroed(size, align) → u8*
ekf2_rust_dealloc(ptr, size, align)
```

These symbols are exported by `ekf2-sys/src/lib.rs` and forward to Rust's
`alloc::alloc::{alloc, alloc_zeroed, dealloc}`.

To carry the `(size, align)` metadata needed for deallocation, `cpp_alloc`
prepends a 16-byte header (two `size_t` fields) before the pointer returned to
the caller:

```
[ raw allocation: align + n bytes          ]
[ size_t align | size_t n | <user data...> ]
                            ^── pointer returned to C++ code
```

`cpp_free` reads back these two fields from `ptr[-2]` and `ptr[-1]` before
calling `ekf2_rust_dealloc`.

## Build

`build.rs` does two things:

1. **Compiles the C++ sources** via the `cc` crate, passing `CONFIG_EKF2_*`
   defines for each enabled Cargo feature.
2. **Runs bindgen** on `ekf2_wrapper.h` to produce `$OUT_DIR/bindings.rs`.
   It also probes the native toolchain to determine `sizeof(Ekf)` at build
   time and writes the result to `$OUT_DIR/constants.rs`.

The vendored PX4 sources must be present under `ekf2-sys/vendor/` before
building. Run `./scripts/vendor.sh` from the workspace root to populate them.

## Feature flags

Each sensor type is guarded by a Cargo feature that maps 1-to-1 to the
corresponding `CONFIG_EKF2_*` C preprocessor define:

| Feature             | Default | C define                        | Purpose                                              |
|---------------------|---------|---------------------------------|------------------------------------------------------|
| `gnss`              | yes     | `CONFIG_EKF2_GNSS`              | GNSS position and velocity fusion                    |
| `magnetometer`      | yes     | `CONFIG_EKF2_MAGNETOMETER`      | Magnetometer heading fusion                          |
| `barometer`         | yes     | `CONFIG_EKF2_BAROMETER`         | Barometric altitude fusion                           |
| `gnss-yaw`          | no      | `CONFIG_EKF2_GNSS_YAW`          | Dual-antenna GNSS yaw fusion                         |
| `baro-compensation` | no      | `CONFIG_EKF2_BARO_COMPENSATION` | Airspeed-based baro error compensation               |
| `airspeed`          | no      | `CONFIG_EKF2_AIRSPEED`          | Pitot-tube airspeed fusion                           |
| `range-finder`      | no      | `CONFIG_EKF2_RANGE_FINDER`      | Downward range-finder altitude fusion (enables terrain) |
| `optical-flow`      | no      | `CONFIG_EKF2_OPTICAL_FLOW`      | Optical-flow velocity fusion (enables terrain)       |
| `external-vision`   | no      | `CONFIG_EKF2_EXTERNAL_VISION`   | External pose/velocity source (VIO, MoCap)           |
| `aux-vel`           | no      | `CONFIG_EKF2_AUXVEL`            | Auxiliary horizontal velocity source                 |
| `drag-fusion`       | no      | `CONFIG_EKF2_DRAG_FUSION`       | Aerodynamic drag-based wind estimation               |
| `wind`              | no      | `CONFIG_EKF2_WIND`              | Wind velocity state estimation                       |
| `terrain`           | no      | `CONFIG_EKF2_TERRAIN`           | Terrain height state (auto-enabled by range/flow)    |
| `gravity-fusion`    | no      | `CONFIG_EKF2_GRAVITY_FUSION`    | Accelerometer gravity vector fusion for attitude     |
| `sideslip`          | no      | `CONFIG_EKF2_SIDESLIP`          | Aerodynamic sideslip fusion (fixed-wing wind est.)   |
| `logging`           | no      | —                               | Enable EKF internal logging output                   |
| `c-stubs`           | no      | —                               | Provide libc shims (malloc/free) on bare-metal       |

Note: `range-finder` and `optical-flow` implicitly enable `CONFIG_EKF2_TERRAIN`.

## no_std

This crate is `#![no_std]` but requires `alloc`. The global allocator must be
configured by the application (e.g. via `embedded-alloc` on bare-metal).
