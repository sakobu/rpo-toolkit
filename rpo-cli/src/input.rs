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

/// Load and parse a JSON file, printing a hint on parse failure.
///
/// On [`CliError::Json`], prints the given hint to stderr before returning
/// the error. This guides users to the expected input schema.
///
/// # Errors
///
/// Same as [`load_json`].
pub fn load_json_with_hint<T: DeserializeOwned>(path: &Path, hint: &str) -> Result<T, CliError> {
    load_json(path).inspect_err(|e| {
        if matches!(e, CliError::Json { .. }) {
            eprintln!("hint: {hint}");
        }
    })
}
