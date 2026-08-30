//! Error types for the EKF2 Rust API.

/// Errors returned by [`Ekf`](crate::Ekf) methods.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EkfError {
    /// Allocating memory for the EKF object failed.
    AllocFailed,

    /// A fallible operation returned `false`.
    OperationFailed,
}

impl core::fmt::Display for EkfError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::AllocFailed => write!(f, "EKF allocation failed"),
            Self::OperationFailed => write!(f, "EKF operation failed"),
        }
    }
}
