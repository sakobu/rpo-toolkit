//! Core domain types: state vectors, Keplerian elements, ROEs, and RIC states.

pub mod eclipse;
pub mod elements;
pub mod roe;
pub mod spacecraft;
pub mod state;

pub use eclipse::*;
pub use elements::*;
pub use roe::*;
pub use spacecraft::*;
pub use state::*;

use nalgebra::SMatrix;

/// 6×6 matrix type alias for STM and covariance operations.
pub type Matrix6 = SMatrix<f64, 6, 6>;

/// 9×9 matrix type alias for the augmented J2+drag STM.
pub type Matrix9 = SMatrix<f64, 9, 9>;

