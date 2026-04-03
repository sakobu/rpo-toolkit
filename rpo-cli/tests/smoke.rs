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
fn mission_default_outputs_markdown_to_stdout() {
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
        stdout.contains("# Mission Summary"),
        "expected markdown summary header on stdout"
    );
}

#[test]
fn mission_json_outputs_to_stdout() {
    let output = cli_cmd()
        .args(["mission", "--input"])
        .arg(examples_dir().join("mission.json"))
        .args(["--json"])
        .output()
        .expect("failed to execute");
    assert!(
        output.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let stdout = String::from_utf8_lossy(&output.stdout);
    let _: serde_json::Value =
        serde_json::from_str(&stdout).expect("stdout should contain valid JSON");
}

#[test]
fn mission_output_flag_writes_to_file() {
    let tmp = std::env::temp_dir().join(format!("rpo-cli-output-test-{}", std::process::id()));
    std::fs::create_dir_all(&tmp).expect("failed to create temp dir");
    let report_path = tmp.join("report.md");
    let output = cli_cmd()
        .args(["mission", "--input"])
        .arg(examples_dir().join("mission.json"))
        .args(["-o"])
        .arg(&report_path)
        .output()
        .expect("failed to execute");
    assert!(
        output.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    // stdout should be empty when writing to file
    assert!(output.stdout.is_empty(), "expected no stdout with -o flag");
    // File should contain markdown
    assert!(report_path.exists(), "expected report file to be created");
    let content = std::fs::read_to_string(&report_path).expect("failed to read report");
    assert!(
        content.contains("# Mission Summary"),
        "expected markdown summary header in file"
    );
    // Clean up
    let _ = std::fs::remove_dir_all(&tmp);
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

#[test]
fn mission_json_output_to_file() {
    let tmp = std::env::temp_dir().join(format!("rpo-cli-json-file-test-{}", std::process::id()));
    std::fs::create_dir_all(&tmp).expect("failed to create temp dir");
    let report_path = tmp.join("report.json");
    let output = cli_cmd()
        .args(["mission", "--input"])
        .arg(examples_dir().join("mission.json"))
        .args(["--json", "-o"])
        .arg(&report_path)
        .output()
        .expect("failed to execute");
    assert!(
        output.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    assert!(report_path.exists(), "expected JSON report file");
    let content = std::fs::read_to_string(&report_path).expect("failed to read report");
    let _: serde_json::Value =
        serde_json::from_str(&content).expect("file should contain valid JSON");
    let _ = std::fs::remove_dir_all(&tmp);
}
