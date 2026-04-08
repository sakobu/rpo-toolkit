//! Typed WASM error type with error codes for structured frontend handling.

use serde::{Deserialize, Serialize};
use tsify_next::Tsify;

use rpo_core::mission::{
    AvoidanceError, EclipseComputeError, FormationDesignError, MissionError,
};
use rpo_core::pipeline::PipelineError;
use rpo_core::propagation::{CovarianceError, PropagationError};

/// Structured WASM error returned to JavaScript.
///
/// Serialized as a JS object with `code`, `message`, and optional `details`.
#[derive(Debug, Clone, Serialize, Deserialize, Tsify)]
#[tsify(into_wasm_abi, from_wasm_abi)]
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
#[derive(Debug, Clone, Serialize, Deserialize, Tsify)]
#[tsify(into_wasm_abi, from_wasm_abi)]
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
    /// Lambert solver error.
    Lambert,
    /// Eclipse computation error.
    Eclipse,
    /// Formation design error.
    Formation,
    /// Input deserialization failed (invalid JSON shape or types).
    Deserialization,
    /// Catch-all for unexpected errors.
    Internal,
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

impl From<EclipseComputeError> for WasmError {
    fn from(e: EclipseComputeError) -> Self {
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
