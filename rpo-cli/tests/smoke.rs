use std::path::PathBuf;
use std::process::Command;

fn examples_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .unwrap()
        .join("examples")
}

fn cli_cmd() -> Command {
    Command::new(env!("CARGO_BIN_EXE_rpo-cli"))
}

#[test]
fn mission_exits_zero() {
    let output = cli_cmd()
        .args(["mission", "--input"])
        .arg(examples_dir().join("mission.json"))
        .output()
        .expect("failed to execute");
    assert!(
        output.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(
        stdout.contains("End-to-End Mission"),
        "expected mission output header"
    );
}

#[test]
fn mission_json_flag_produces_valid_json() {
    let output = cli_cmd()
        .args(["mission", "--input"])
        .arg(examples_dir().join("mission.json"))
        .args(["--json"])
        .output()
        .expect("failed to execute");
    assert!(output.status.success());
    let stdout = String::from_utf8_lossy(&output.stdout);
    let _: serde_json::Value =
        serde_json::from_str(&stdout).expect("--json should produce valid JSON");
}

#[test]
fn classify_exits_zero() {
    let output = cli_cmd()
        .args(["classify", "--input"])
        .arg(examples_dir().join("mission.json"))
        .output()
        .expect("failed to execute");
    assert!(output.status.success());
}

#[test]
fn nonexistent_input_returns_nonzero() {
    let output = cli_cmd()
        .args(["mission", "--input", "nonexistent.json"])
        .output()
        .expect("failed to execute");
    assert!(!output.status.success());
}

#[test]
fn no_args_exits_zero() {
    // When no subcommand is given, main.rs calls Cli::parse_from(["rpo-cli", "--help"])
    // which triggers clap's help printer + process::exit(0). Just verify clean exit.
    let output = cli_cmd().output().expect("failed to execute");
    assert!(output.status.success());
}
