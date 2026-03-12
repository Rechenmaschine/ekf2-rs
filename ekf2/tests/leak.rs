use std::alloc::{GlobalAlloc, Layout, System};
use std::sync::atomic::{AtomicIsize, Ordering::Relaxed};

use ekf2::{types::ImuSample, Ekf, EkfError};

#[cfg(feature = "barometer")]
use ekf2::types::BaroSample;
#[cfg(feature = "gnss")]
use ekf2::types::GnssSample;
#[cfg(feature = "magnetometer")]
use ekf2::types::MagSample;

// ── Tracking allocator ───────────────────────────────────────────────────────

struct CountingAllocator;

static BALANCE: AtomicIsize = AtomicIsize::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        let ptr = unsafe { System.alloc(layout) };
        if !ptr.is_null() {
            BALANCE.fetch_add(layout.size() as isize, Relaxed);
        }
        ptr
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        unsafe { System.dealloc(ptr, layout) };
        BALANCE.fetch_sub(layout.size() as isize, Relaxed);
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        let new_ptr = unsafe { System.realloc(ptr, layout, new_size) };
        if !new_ptr.is_null() {
            BALANCE.fetch_add(new_size as isize - layout.size() as isize, Relaxed);
        }
        new_ptr
    }
}

#[global_allocator]
static A: CountingAllocator = CountingAllocator;

// ── Helpers ──────────────────────────────────────────────────────────────────

fn run_imu_cycle(ts_base: u64, steps: u64) {
    let mut ekf = Ekf::new(ts_base).expect("init should succeed");
    let dt_s = 0.01_f32;
    for i in 0..steps {
        let ts = ts_base + 100_000 + i * 10_000;
        let imu = ImuSample::new(ts, [0.0, 0.0, 0.0002], [0.0, 0.0, -9.81 * dt_s], dt_s, dt_s);
        ekf.set_imu_data(&imu);
        assert!(matches!(ekf.update(), Ok(()) | Err(EkfError::UpdateFailed)));
    }
}

#[cfg(all(feature = "gnss", feature = "magnetometer", feature = "barometer"))]
fn run_sensor_cycle(ts_base: u64, steps: u64) {
    let mut ekf = Ekf::new(ts_base).expect("init should succeed");
    let dt_s = 0.01_f32;
    for i in 0..steps {
        let ts = ts_base + 100_000 + i * 10_000;
        let imu = ImuSample::new(ts, [0.0, 0.0, 0.0003], [0.0, 0.0, -9.81 * dt_s], dt_s, dt_s);
        ekf.set_imu_data(&imu);

        if i % 10 == 0 {
            ekf.set_baro_data(&BaroSample::new(ts, 488.0));
            ekf.set_mag_data(&MagSample::new(ts, [0.21, 0.0, 0.43]));
            ekf.set_gps_data(&GnssSample::new(
                ts,
                47.397742,
                8.545594,
                488.0,
                [0.1, 0.0, 0.0],
                0.5,
                0.8,
                0.2,
                3,
                16,
            ));
        }

        assert!(matches!(ekf.update(), Ok(()) | Err(EkfError::UpdateFailed)));
    }
}

// ── Test ─────────────────────────────────────────────────────────────────────

#[test]
fn create_drop_cycles_do_not_leak() {
    const WARMUP: usize = 10;
    const CYCLES: usize = 200;

    // ── Phase 1: IMU-only ────────────────────────────────────────────────
    {
        let mut samples = Vec::with_capacity(CYCLES);

        for i in 0..WARMUP {
            run_imu_cycle(i as u64 * 2_000_000, 100);
        }

        for i in 0..CYCLES {
            run_imu_cycle((WARMUP + i) as u64 * 2_000_000, 100);
            samples.push(BALANCE.load(Relaxed));
        }

        assert_no_drift(&samples, "imu-only");
    }

    // ── Phase 2: with aiding sensors ─────────────────────────────────────
    #[cfg(all(feature = "gnss", feature = "magnetometer", feature = "barometer"))]
    {
        let mut samples = Vec::with_capacity(CYCLES);

        for i in 0..WARMUP {
            run_sensor_cycle(i as u64 * 3_000_000, 120);
        }

        for i in 0..CYCLES {
            run_sensor_cycle((WARMUP + i) as u64 * 3_000_000, 120);
            samples.push(BALANCE.load(Relaxed));
        }

        assert_no_drift(&samples, "sensor-rich");
    }
}

fn assert_no_drift(samples: &[isize], label: &str) {
    let n = samples.len();
    assert!(n >= 20, "need at least 20 samples for drift detection");

    let min = *samples.iter().min().unwrap();
    let max = *samples.iter().max().unwrap();
    let span = max - min;

    assert!(
        span < 16_384,
        "{label}: allocation balance span too large ({span} bytes, min={min}, max={max}), \
         likely leak"
    );

    let q1 = median(&samples[..n / 4]);
    let q4 = median(&samples[3 * n / 4..]);
    let drift = q4 - q1;

    assert!(
        drift < 4_096,
        "{label}: allocation balance drifting upward (q1 median={q1}, q4 median={q4}, \
         drift={drift} bytes), likely leak"
    );
}

fn median(slice: &[isize]) -> isize {
    let mut sorted = slice.to_vec();
    sorted.sort();
    sorted[sorted.len() / 2]
}
