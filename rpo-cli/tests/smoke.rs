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
        stdout.contains("Mission\n==="),
        "expected mission output header"
    );
}

#[test]
fn mission_json_flag_produces_json_file() {
    let tmp = std::env::temp_dir().join(format!("rpo-cli-json-test-{}", std::process::id()));
    std::fs::create_dir_all(&tmp).expect("failed to create temp dir");
    let output = cli_cmd()
        .current_dir(&tmp)
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
    // JSON now writes to file, not stdout
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.trim().is_empty(), "expected no stdout output with --json");
    let report_path = tmp.join("reports/json/mission.json");
    assert!(report_path.exists(), "expected reports/json/mission.json to be created");
    let content = std::fs::read_to_string(&report_path).expect("failed to read report");
    let _: serde_json::Value =
        serde_json::from_str(&content).expect("report should contain valid JSON");
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
fn mission_markdown_flag_produces_markdown() {
    // Use a unique temp directory so the test doesn't pollute the workspace.
    let tmp = std::env::temp_dir().join(format!("rpo-cli-test-{}", std::process::id()));
    std::fs::create_dir_all(&tmp).expect("failed to create temp dir");
    let output = cli_cmd()
        .current_dir(&tmp)
        .args(["mission", "--input"])
        .arg(examples_dir().join("mission.json"))
        .args(["--markdown"])
        .output()
        .expect("failed to execute");
    assert!(
        output.status.success(),
        "stderr: {}",
        String::from_utf8_lossy(&output.stderr)
    );
    let report_path = tmp.join("reports/markdown/mission.md");
    assert!(report_path.exists(), "expected reports/markdown/mission.md to be created");
    let content = std::fs::read_to_string(&report_path).expect("failed to read report");
    assert!(
        content.contains("# Mission Summary"),
        "expected markdown summary header"
    );
    assert!(
        content.contains("**Verdict:"),
        "expected verdict line"
    );
    // No duplicate summary: only one H1 heading
    let h1_count = content.lines().filter(|l| l.starts_with("# ")).count();
    assert_eq!(
        h1_count, 1,
        "expected exactly one H1 heading (# Mission Summary), got {h1_count}"
    );
    // Clean up
    let _ = std::fs::remove_dir_all(&tmp);
}

#[test]
fn json_and_markdown_conflict() {
    let output = cli_cmd()
        .args(["mission", "--input"])
        .arg(examples_dir().join("mission.json"))
        .args(["--json", "--markdown"])
        .output()
        .expect("failed to execute");
    assert!(
        !output.status.success(),
        "expected non-zero exit for conflicting flags"
    );
}

#[test]
fn validate_json_and_markdown_conflict() {
    let output = cli_cmd()
        .args(["validate", "--input"])
        .arg(examples_dir().join("validate.json"))
        .args(["--json", "--markdown"])
        .output()
        .expect("failed to execute");
    assert!(
        !output.status.success(),
        "expected non-zero exit for conflicting flags"
    );
}

#[test]
fn mc_json_and_markdown_conflict() {
    let output = cli_cmd()
        .args(["mc", "--input"])
        .arg(examples_dir().join("mc.json"))
        .args(["--json", "--markdown"])
        .output()
        .expect("failed to execute");
    assert!(
        !output.status.success(),
        "expected non-zero exit for conflicting flags"
    );
}
