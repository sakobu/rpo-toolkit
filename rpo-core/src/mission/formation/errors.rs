//! Error types for formation design operations.

use serde::{Deserialize, Serialize};

use crate::elements::keplerian_conversions::ConversionError;
use crate::mission::safety::SafetyError;
use crate::propagation::propagator::PropagationError;
use crate::types::KeplerError;

/// Errors from formation design operations.
#[derive(Debug, Clone)]
pub enum FormationDesignError {
    /// `T_pos` null-space computation produced degenerate geometry.
    SingularGeometry {
        /// Chief mean argument of latitude where singularity occurs (rad).
        mean_arg_lat_rad: f64,
    },
    /// Requested `d_min` cannot be achieved within linearization bounds.
    SeparationUnachievable {
        /// Requested separation (km).
        requested_km: f64,
        /// Maximum achievable separation within linearization bound (km).
        achievable_km: f64,
    },
    /// Chief mean elements are invalid (e.g., `a_km` <= 0, e >= 1).
    InvalidChiefElements(ConversionError),
    /// Safety analysis computation failed.
    SafetyAnalysis(SafetyError),
    /// Transit trajectory has insufficient sampling density for reliable e/i monitoring.
    InsufficientSampling {
        /// Total samples in the provided trajectory.
        total_samples: u32,
        /// Required minimum samples per orbital period.
        required_per_orbit: u32,
    },
    /// J2 parameter computation failed.
    Propagation(PropagationError),
    /// Kepler equation or derived-quantity failure (period, mean motion).
    KeplerFailure(KeplerError),
}

impl std::fmt::Display for FormationDesignError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::SingularGeometry { mean_arg_lat_rad } => write!(
                f,
                "FormationDesignError: singular T_pos geometry at mean arg lat = {mean_arg_lat_rad:.6} rad"
            ),
            Self::SeparationUnachievable {
                requested_km,
                achievable_km,
            } => write!(
                f,
                "FormationDesignError: requested separation {requested_km:.6} km \
                 exceeds achievable {achievable_km:.6} km within linearization bounds"
            ),
            Self::InvalidChiefElements(e) => {
                write!(f, "FormationDesignError: invalid chief elements — {e}")
            }
            Self::SafetyAnalysis(e) => {
                write!(f, "FormationDesignError: safety analysis — {e}")
            }
            Self::InsufficientSampling {
                total_samples,
                required_per_orbit,
            } => write!(
                f,
                "FormationDesignError: {total_samples} total samples < {required_per_orbit} required per orbit"
            ),
            Self::Propagation(e) => {
                write!(f, "FormationDesignError: propagation — {e}")
            }
            Self::KeplerFailure(e) => {
                write!(f, "FormationDesignError: kepler failure — {e}")
            }
        }
    }
}

impl std::error::Error for FormationDesignError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::InvalidChiefElements(e) => Some(e),
            Self::SafetyAnalysis(e) => Some(e),
            Self::Propagation(e) => Some(e),
            Self::KeplerFailure(e) => Some(e),
            _ => None,
        }
    }
}

impl From<ConversionError> for FormationDesignError {
    fn from(e: ConversionError) -> Self {
        Self::InvalidChiefElements(e)
    }
}

impl From<SafetyError> for FormationDesignError {
    fn from(e: SafetyError) -> Self {
        Self::SafetyAnalysis(e)
    }
}

impl From<PropagationError> for FormationDesignError {
    fn from(e: PropagationError) -> Self {
        Self::Propagation(e)
    }
}

impl From<KeplerError> for FormationDesignError {
    fn from(e: KeplerError) -> Self {
        Self::KeplerFailure(e)
    }
}

/// Structured reason for perch enrichment fallback.
///
/// Serializable projection of [`FormationDesignError`] — structured for common
/// variants, with `detail` strings only for rare wrapped-inner-error cases
/// where the inner types don't implement `Serialize`.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum PerchFallbackReason {
    /// `T_pos` null-space produced degenerate geometry at this mean argument of latitude.
    SingularGeometry {
        /// Mean argument of latitude where singularity occurs (rad).
        mean_arg_lat_rad: f64,
    },
    /// Requested separation exceeds achievable within linearization bounds.
    SeparationUnachievable {
        /// Requested separation (km).
        requested_km: f64,
        /// Maximum achievable separation (km).
        achievable_km: f64,
    },
    /// Chief mean elements are invalid.
    InvalidChiefElements {
        /// Display string from the inner `ConversionError`.
        detail: String,
    },
    /// Safety analysis computation failed.
    SafetyAnalysis {
        /// Display string from the inner `SafetyError`.
        detail: String,
    },
    /// J2 parameter computation failed.
    Propagation {
        /// Display string from the inner `PropagationError`.
        detail: String,
    },
    /// Kepler equation or derived-quantity failure.
    KeplerFailure {
        /// Display string from the inner `KeplerError`.
        detail: String,
    },
    /// Transit trajectory has insufficient sampling density.
    /// Cannot occur during perch enrichment — included for exhaustive mapping.
    InsufficientSampling {
        /// Total samples in the provided trajectory.
        total_samples: u32,
        /// Required minimum samples per orbital period.
        required_per_orbit: u32,
    },
}

impl From<FormationDesignError> for PerchFallbackReason {
    fn from(e: FormationDesignError) -> Self {
        match e {
            FormationDesignError::SingularGeometry { mean_arg_lat_rad } => {
                Self::SingularGeometry { mean_arg_lat_rad }
            }
            FormationDesignError::SeparationUnachievable {
                requested_km,
                achievable_km,
            } => Self::SeparationUnachievable {
                requested_km,
                achievable_km,
            },
            FormationDesignError::InvalidChiefElements(inner) => {
                Self::InvalidChiefElements {
                    detail: inner.to_string(),
                }
            }
            FormationDesignError::SafetyAnalysis(inner) => Self::SafetyAnalysis {
                detail: inner.to_string(),
            },
            FormationDesignError::InsufficientSampling {
                total_samples,
                required_per_orbit,
            } => Self::InsufficientSampling {
                total_samples,
                required_per_orbit,
            },
            FormationDesignError::Propagation(inner) => Self::Propagation {
                detail: inner.to_string(),
            },
            FormationDesignError::KeplerFailure(inner) => Self::KeplerFailure {
                detail: inner.to_string(),
            },
        }
    }
}
