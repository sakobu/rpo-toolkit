//! CLI error type and exit codes.

use std::fmt;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use rpo_core::elements::keplerian_conversions::ConversionError;
use rpo_core::mission::MissionError;
use rpo_core::propagation::{
    CovarianceError, LambertError, PropagationError,
};
use rpo_nyx::monte_carlo::MonteCarloError as NyxMonteCarloError;
use rpo_nyx::nyx_bridge::NyxBridgeError;
use rpo_nyx::pipeline::PipelineError as NyxPipelineError;
use rpo_nyx::validation::ValidationError;

/// Unified CLI error type.
#[derive(Debug)]
pub enum CliError {
    /// File I/O error.
    Io {
        /// Path to the file that caused the error.
        path: PathBuf,
        /// Underlying I/O error.
        source: std::io::Error,
    },
    /// JSON parsing error.
    Json {
        /// Path to the file that caused the error.
        path: PathBuf,
        /// Underlying `serde_json` error.
        source: serde_json::Error,
    },
    /// Pipeline error (planning, propagation, nyx operations, etc.).
    Pipeline(NyxPipelineError),
    /// JSON serialization error (output).
    Serialize(serde_json::Error),
    /// A required field is missing for the requested operation.
    MissingField {
        /// Name of the missing field.
        field: &'static str,
        /// Subcommand or operation that requires it.
        context: &'static str,
    },
    /// Epoch string could not be parsed.
    EpochParse {
        /// The input string that failed to parse.
        input: String,
        /// The parse error detail.
        source: String,
    },
    /// Unknown output format requested.
    UnknownFormat {
        /// The format string the user provided.
        format: String,
        /// List of valid formats.
        valid: &'static [&'static str],
    },
}

impl fmt::Display for CliError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => {
                write!(f, "I/O error: {}: {source}", path.display())
            }
            Self::Json { path, source } => {
                write!(f, "failed to parse {}: {source}", path.display())
            }
            Self::Pipeline(e) => write!(f, "{e}"),
            Self::Serialize(e) => write!(f, "JSON serialization failed: {e}"),
            Self::MissingField { field, context } => {
                write!(f, "'{context}' subcommand requires '{field}' in input JSON")
            }
            Self::EpochParse { input, source } => {
                write!(f, "invalid epoch '{input}': {source}")
            }
            Self::UnknownFormat { format, valid } => {
                write!(f, "unknown target format: {format}. Valid: {}", valid.join(", "))
            }
        }
    }
}

impl std::error::Error for CliError {}

impl From<NyxPipelineError> for CliError {
    fn from(e: NyxPipelineError) -> Self {
        Self::Pipeline(e)
    }
}

impl From<rpo_core::pipeline::PipelineError> for CliError {
    fn from(e: rpo_core::pipeline::PipelineError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<MissionError> for CliError {
    fn from(e: MissionError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<ConversionError> for CliError {
    fn from(e: ConversionError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<LambertError> for CliError {
    fn from(e: LambertError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<PropagationError> for CliError {
    fn from(e: PropagationError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<NyxBridgeError> for CliError {
    fn from(e: NyxBridgeError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<ValidationError> for CliError {
    fn from(e: ValidationError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<CovarianceError> for CliError {
    fn from(e: CovarianceError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl From<NyxMonteCarloError> for CliError {
    fn from(e: NyxMonteCarloError) -> Self {
        Self::Pipeline(NyxPipelineError::from(e))
    }
}

impl CliError {
    /// Create an I/O error with the given path context.
    pub(crate) fn io(path: &Path, source: std::io::Error) -> Self {
        Self::Io {
            path: path.to_path_buf(),
            source,
        }
    }

    /// Map this error to an exit code.
    pub fn exit_code(&self) -> ExitCode {
        match self {
            Self::Io { .. }
            | Self::Json { .. }
            | Self::MissingField { .. }
            | Self::EpochParse { .. }
            | Self::UnknownFormat { .. } => ExitCode::from(1),
            Self::Pipeline(_) | Self::Serialize(_) => ExitCode::from(2),
        }
    }
}
