/// Demonstrates Ekf ownership: heap-allocated via Ekf::new(), movable, and box-able.
use ekf2::{types::ImuSample, Ekf, EkfError};

fn main() -> Result<(), EkfError> {
    // Basic heap-allocated instance.
    let mut ekf = Ekf::new(0)?;
    let imu = ImuSample::new(10_000, [0.0, 0.0, 0.0002], [0.0, 0.0, -0.0981], 0.01, 0.01);
    ekf.set_imu_data(&imu);
    let _ = ekf.update();

    // Ekf is movable — ownership transfers without copying the heap allocation.
    let mut moved = ekf;
    let imu2 = ImuSample::new(20_000, [0.0, 0.0, 0.0002], [0.0, 0.0, -0.0981], 0.01, 0.01);
    moved.set_imu_data(&imu2);
    let _ = moved.update();
    println!("after move — q: {:?}", moved.quaternion());

    // Ekf can be boxed like any heap type.
    let boxed: Box<Ekf> = Box::new(Ekf::new(0)?);
    println!("boxed q: {:?}", boxed.quaternion());

    // Multiple independent instances on the heap.
    let ekf_a = Ekf::new(0)?;
    let ekf_b = Ekf::new(0)?;
    println!("a q: {:?}", ekf_a.quaternion());
    println!("b q: {:?}", ekf_b.quaternion());

    Ok(())
}
