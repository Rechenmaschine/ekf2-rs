//! Error types for the EKF2 Rust API.

/// Errors returned by [`Ekf`](crate::Ekf) methods.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EkfError {
    /// Allocating memory for the EKF object failed.
    AllocFailed,

    /// The C++ `Ekf::init()` returned `false`.
    InitFailed,

    /// The C++ `Ekf::update()` returned `false`.
    UpdateFailed,
}

impl core::fmt::Display for EkfError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::AllocFailed => write!(f, "EKF allocation failed"),
            Self::InitFailed => write!(f, "Ekf::init() returned false"),
            Self::UpdateFailed => write!(f, "Ekf::update() returned false"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::EkfError;

    #[test]
    fn display_messages_are_stable() {
        extern crate alloc;
        use alloc::string::ToString;
        assert_eq!(EkfError::AllocFailed.to_string(), "EKF allocation failed");
        assert_eq!(
            EkfError::InitFailed.to_string(),
            "Ekf::init() returned false"
        );
        assert_eq!(
            EkfError::UpdateFailed.to_string(),
            "Ekf::update() returned false"
        );
    }
}
