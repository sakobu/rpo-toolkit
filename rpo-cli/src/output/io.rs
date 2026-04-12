//! I/O and progress display helpers.

use std::path::Path;
use std::time::Duration;

use indicatif::{ProgressBar, ProgressStyle};
use serde::Serialize;

use crate::error::CliError;

/// Spinner animation tick interval (milliseconds).
const SPINNER_TICK_MS: u64 = 120;

/// Print a status message to the spinner if available, otherwise to stderr.
macro_rules! status {
    ($spinner:expr, $($arg:tt)*) => {
        if let Some(s) = &$spinner {
            s.set_message(format!($($arg)*));
        } else {
            eprintln!($($arg)*);
        }
    };
}
pub(crate) use status;

/// Create an animated spinner for long-running CLI operations.
///
/// The spinner writes to stderr, so it does not interfere with stdout output.
/// Returns `None` if stderr is not a terminal (piped/redirected).
#[must_use]
pub fn create_spinner() -> Option<ProgressBar> {
    if !std::io::IsTerminal::is_terminal(&std::io::stderr()) {
        return None;
    }
    let s = ProgressBar::new_spinner();
    s.set_style(
        ProgressStyle::default_spinner()
            .template("{spinner:.green} {msg}")
            .unwrap_or_else(|_| ProgressStyle::default_spinner()),
    );
    s.enable_steady_tick(Duration::from_millis(SPINNER_TICK_MS));
    Some(s)
}

/// Create the output sink: a buffered file writer, or locked stdout.
fn open_output(path: Option<&Path>) -> Result<Box<dyn std::io::Write>, CliError> {
    if let Some(p) = path {
        if let Some(parent) = p.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent).map_err(|e| CliError::io(parent, e))?;
            }
        }
        let file = std::fs::File::create(p).map_err(|e| CliError::io(p, e))?;
        Ok(Box::new(std::io::BufWriter::new(file)))
    } else {
        Ok(Box::new(std::io::stdout().lock()))
    }
}

/// Write text content to a file or stdout.
///
/// If `path` is `Some`, writes to that file (creating parent directories)
/// and prints the path to stderr. If `None`, prints to stdout.
///
/// # Errors
///
/// Returns [`CliError::Io`] if directory creation or file write fails.
pub fn output_text(content: &str, path: Option<&Path>) -> Result<(), CliError> {
    use std::io::Write;
    let mut out = open_output(path)?;
    out.write_all(content.as_bytes())
        .map_err(|e| CliError::io(path.unwrap_or(Path::new("<stdout>")), e))?;
    if let Some(p) = path {
        eprintln!("Written to {}", p.display());
    }
    Ok(())
}

/// Serialize a value as JSON directly to a file or stdout.
///
/// Writes directly to the output stream via [`serde_json::to_writer_pretty`],
/// avoiding an intermediate `String` allocation.
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails, or [`CliError::Io`]
/// if file write fails.
pub fn output_json<T: Serialize>(data: &T, path: Option<&Path>) -> Result<(), CliError> {
    use std::io::Write;
    let mut out = open_output(path)?;
    serde_json::to_writer_pretty(&mut out, data).map_err(CliError::Serialize)?;
    out.write_all(b"\n")
        .map_err(|e| CliError::io(path.unwrap_or(Path::new("<stdout>")), e))?;
    if let Some(p) = path {
        eprintln!("Written to {}", p.display());
    }
    Ok(())
}

/// Pretty-print a value as JSON to stdout (plumbing commands).
///
/// # Errors
///
/// Returns [`CliError::Serialize`] if serialization fails.
pub fn print_json<T: Serialize>(value: &T) -> Result<(), CliError> {
    output_json(value, None)
}
