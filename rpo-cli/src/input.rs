//! Input loading helpers.

use std::path::Path;

use serde::de::DeserializeOwned;

use crate::error::CliError;

/// Load and parse a JSON file into the given type.
///
/// # Errors
///
/// Returns [`CliError::Io`] if the file cannot be read,
/// or [`CliError::Json`] if the JSON is malformed.
pub fn load_json<T: DeserializeOwned>(path: &Path) -> Result<T, CliError> {
    let contents = std::fs::read_to_string(path).map_err(|e| CliError::Io {
        path: path.to_path_buf(),
        source: e,
    })?;
    serde_json::from_str(&contents).map_err(|e| CliError::Json {
        path: path.to_path_buf(),
        source: e,
    })
}
