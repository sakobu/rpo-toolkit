//! Markdown report generation.
//!
//! Pure functions that produce self-contained markdown reports as `String`s.
//! Called by porcelain command `run()` functions in Summary output mode.
//! No `println!` --- the caller writes the returned string via `output_text`.
//!
//! # Note on `let _ = writeln!(...)`
//!
//! `writeln!` to `String` is infallible (`fmt::Write` for `String` always
//! returns `Ok`). The `let _ =` suppresses the unused `Result` lint without
//! introducing an `unwrap()` or `expect()` that would imply fallibility.
//! This is a standard Rust convention for infallible formatting.

mod helpers;
mod mc;
mod mission;

pub use mc::mc_to_markdown;
pub use mission::{mission_to_markdown, validation_to_markdown, ValidationContext};
