//! Input loading helpers.

use std::path::Path;

use serde::de::DeserializeOwned;

use crate::error::CliError;

/// Load and parse a JSON file into the given type.
///
/// When `hint` is `Some`, prints the hint to stderr on parse failure
/// to guide users toward the expected input schema.
///
/// # Errors
///
/// Returns [`CliError::Io`] if the file cannot be read,
/// or [`CliError::Json`] if the JSON is malformed.
pub fn load_json<T: DeserializeOwned>(path: &Path, hint: Option<&str>) -> Result<T, CliError> {
    let contents = std::fs::read_to_string(path).map_err(|e| CliError::io(path, e))?;
    serde_json::from_str(&contents).map_err(|e| {
        if let Some(hint) = hint {
            eprintln!("hint: {hint}");
        }
        CliError::Json {
            path: path.to_path_buf(),
            source: e,
        }
    })
}
