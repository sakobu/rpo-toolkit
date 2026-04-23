//! Typed WASM error type with error codes for structured frontend handling.

use serde::Serialize;
use tsify_next::Tsify;

use rpo_core::elements::eci_ric_dcm::DcmError;
use rpo_core::elements::eclipse::EclipseGeometryError;
use rpo_core::elements::geodetic::GeodeticError;
use rpo_core::elements::ConversionError;
use rpo_core::mission::{AvoidanceError, FormationDesignError, MissionEclipseError, MissionError};
use rpo_core::pipeline::PipelineError;
use rpo_core::propagation::{CovarianceError, PropagationError};

/// Structured WASM error returned to JavaScript.
///
/// Serialized as a JS object with `code`, `message`, and optional `details`.
#[derive(Debug, Clone, Serialize, Tsify)]
#[tsify(into_wasm_abi)]
#[cfg_attr(test, derive(serde::Deserialize))]
pub struct WasmError {
    /// Machine-readable error category.
    pub code: WasmErrorCode,
    /// Human-readable error description.
    pub message: String,
    /// Optional additional context (e.g. source error chain).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub details: Option<String>,
}

/// Machine-readable error codes for frontend dispatch.
#[derive(Debug, Clone, Serialize, Tsify)]
#[tsify(into_wasm_abi)]
#[cfg_attr(test, derive(serde::Deserialize))]
#[serde(rename_all = "snake_case")]
pub enum WasmErrorCode {
    /// Mission planning error (classification, targeting, waypoints).
    Mission,
    /// Propagation error (STM, Keplerian).
    Propagation,
    /// Covariance propagation error.
    Covariance,
    /// Collision avoidance maneuver error.
    Avoidance,
    /// A required field is missing.
    MissingField,
    /// Trajectory data is empty.
    EmptyTrajectory,
    /// Eclipse computation error.
    Eclipse,
    /// Formation design error.
    Formation,
    /// Input deserialization failed (invalid JSON shape or types).
    Deserialization,
    /// Frame-transform error (geodetic conversion, epoch parse, ECI↔ECEF).
    Frame,
}

impl WasmError {
    /// Construct a frame-transform error with the given message.
    ///
    /// Used at the WASM boundary for input validation (epoch parse, vector
    /// length, etc.) before delegating to a typed `rpo-core` error.
    #[must_use]
    pub fn frame(message: impl Into<String>) -> Self {
        Self {
            code: WasmErrorCode::Frame,
            message: message.into(),
            details: None,
        }
    }
}

impl From<PipelineError> for WasmError {
    fn from(e: PipelineError) -> Self {
        let code = match &e {
            PipelineError::Mission(_) => WasmErrorCode::Mission,
            PipelineError::Propagation(_) => WasmErrorCode::Propagation,
            PipelineError::Covariance(_) => WasmErrorCode::Covariance,
            PipelineError::MissingField { .. } => WasmErrorCode::MissingField,
            PipelineError::EmptyTrajectory => WasmErrorCode::EmptyTrajectory,
        };
        Self {
            code,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<MissionError> for WasmError {
    fn from(e: MissionError) -> Self {
        Self {
            code: WasmErrorCode::Mission,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<PropagationError> for WasmError {
    fn from(e: PropagationError) -> Self {
        Self {
            code: WasmErrorCode::Propagation,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<CovarianceError> for WasmError {
    fn from(e: CovarianceError) -> Self {
        Self {
            code: WasmErrorCode::Covariance,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<AvoidanceError> for WasmError {
    fn from(e: AvoidanceError) -> Self {
        Self {
            code: WasmErrorCode::Avoidance,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<EclipseGeometryError> for WasmError {
    fn from(e: EclipseGeometryError) -> Self {
        Self {
            code: WasmErrorCode::Eclipse,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<MissionEclipseError> for WasmError {
    fn from(e: MissionEclipseError) -> Self {
        Self {
            code: WasmErrorCode::Eclipse,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<FormationDesignError> for WasmError {
    fn from(e: FormationDesignError) -> Self {
        Self {
            code: WasmErrorCode::Formation,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<GeodeticError> for WasmError {
    fn from(e: GeodeticError) -> Self {
        Self {
            code: WasmErrorCode::Frame,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<DcmError> for WasmError {
    fn from(e: DcmError) -> Self {
        Self {
            code: WasmErrorCode::Frame,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

impl From<ConversionError> for WasmError {
    fn from(e: ConversionError) -> Self {
        Self {
            code: WasmErrorCode::Frame,
            message: e.to_string(),
            details: std::error::Error::source(&e).map(ToString::to_string),
        }
    }
}

/// Deserialize a [`wasm_bindgen::JsValue`] into a typed Rust value.
///
/// Centralizes the `serde_wasm_bindgen` conversion and error mapping so that
/// callers using `JsValue` parameters (bare arrays, nested `Vec<Vec<T>>`)
/// get consistent, descriptive error messages.
///
/// # Errors
///
/// Returns [`WasmError`] with [`WasmErrorCode::Deserialization`] if the
/// value cannot be deserialized into the target type.
pub fn deserialize_js<T: serde::de::DeserializeOwned>(
    value: wasm_bindgen::JsValue,
    field: &str,
) -> Result<T, WasmError> {
    serde_wasm_bindgen::from_value(value).map_err(|e| WasmError {
        code: WasmErrorCode::Deserialization,
        message: format!("failed to deserialize {field}: {e}"),
        details: None,
    })
}

/// Parse a hifitime-compatible epoch string into an [`hifitime::Epoch`].
///
/// Accepts the same formats as the canonical `StateVector` deserializer
/// (`epoch_serde::deserialize` in `rpo-core::types::state`), which uses
/// `Epoch::from_gregorian_str`: ISO 8601 with optional timescale suffix
/// (`UTC | TAI | TDB | TT | ET | GPS`) or `Z` / `±HH:MM` offset.
///
/// # Errors
///
/// Returns [`WasmError`] with [`WasmErrorCode::Frame`] if the epoch string
/// cannot be parsed.
pub fn parse_epoch(s: &str) -> Result<hifitime::Epoch, WasmError> {
    hifitime::Epoch::from_gregorian_str(s).map_err(|e| WasmError {
        code: WasmErrorCode::Frame,
        message: format!("invalid epoch string {s:?}: {e}"),
        details: std::error::Error::source(&e).map(ToString::to_string),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pipeline_missing_field_maps_to_missing_field_code() {
        let err = PipelineError::MissingField {
            field: "chief",
            context: "classification",
        };
        let wasm_err = WasmError::from(err);
        assert!(matches!(wasm_err.code, WasmErrorCode::MissingField));
        assert!(wasm_err.message.contains("chief"));
    }

    #[test]
    fn pipeline_empty_trajectory_maps_correctly() {
        let err = PipelineError::EmptyTrajectory;
        let wasm_err = WasmError::from(err);
        assert!(matches!(wasm_err.code, WasmErrorCode::EmptyTrajectory));
    }

    #[test]
    fn propagation_error_maps_to_propagation_code() {
        let err = PropagationError::ZeroSteps;
        let wasm_err = WasmError::from(err);
        assert!(matches!(wasm_err.code, WasmErrorCode::Propagation));
    }
}
