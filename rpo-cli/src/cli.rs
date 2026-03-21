//! CLI definition: clap parser and command enum.

use std::path::PathBuf;

use clap::{ColorChoice, Parser, Subcommand};

/// RPO mission planning tool.
#[derive(Parser)]
#[command(
    name = "rpo-cli",
    about = "RPO mission planning tool",
    after_help = "Porcelain commands produce human-readable output by default (use --json for machine output).\n\
                  Plumbing commands always produce JSON."
)]
pub struct Cli {
    /// Color output mode.
    #[arg(long, global = true, default_value_t = ColorChoice::Auto)]
    pub color: ColorChoice,

    /// Subcommand to run.
    #[command(subcommand)]
    pub command: Option<Command>,
}

/// CLI commands: porcelain (human-readable) and plumbing (JSON-only).
#[derive(Subcommand)]
pub enum Command {
    // ---- Porcelain ----
    /// End-to-end mission: classification → Lambert transfer → perch → waypoint targeting.
    Mission {
        /// Path to JSON input file with chief/deputy states, perch, and waypoints.
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text.
        #[arg(long)]
        json: bool,
    },
    /// End-to-end mission with Nyx high-fidelity validation (requires network on first run).
    Validate {
        /// Path to JSON input file with chief/deputy states, perch, waypoints, and spacecraft configs.
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text.
        #[arg(long)]
        json: bool,
        /// Number of sample points per leg for validation comparison.
        #[arg(long, default_value_t = 50)]
        samples_per_leg: u32,
        /// Auto-derive differential drag config from spacecraft properties via Nyx.
        #[arg(long)]
        auto_drag: bool,
    },
    /// Full-physics Monte Carlo ensemble analysis (requires network on first run).
    Mc {
        /// Path to JSON input file.
        #[arg(short, long)]
        input: PathBuf,
        /// Output as JSON instead of human-readable text.
        #[arg(long)]
        json: bool,
        /// Auto-derive differential drag config from spacecraft properties via Nyx.
        #[arg(long)]
        auto_drag: bool,
    },

    // ---- Plumbing ----
    /// Classify chief/deputy separation (JSON only).
    #[command(after_help = "Input: { chief: StateVector, deputy: StateVector, proximity?: ProximityConfig }\nExample: examples/classify.json")]
    Classify {
        /// Path to JSON input file (see examples/classify.json).
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Solve Lambert transfer (JSON only).
    #[command(after_help = "Input: { departure: StateVector, arrival: StateVector, config?: LambertConfig }\nTOF is derived from the epoch difference between departure and arrival.\nExample: examples/transfer.json")]
    Transfer {
        /// Path to JSON input file (see examples/transfer.json).
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Convert between state representations (JSON only).
    #[command(after_help = "Input: a bare StateVector { epoch, position_eci_km, velocity_eci_km_s }\nExample: examples/convert.json")]
    Convert {
        /// Path to JSON input file (see examples/convert.json).
        #[arg(short, long)]
        input: PathBuf,
        /// Target representation: keplerian.
        #[arg(long, value_name = "FORMAT")]
        to: String,
    },
    /// Propagate ROE state via STM (JSON only).
    #[command(after_help = "Input: { roe: ROE, chief_mean: KeplerianElements, epoch: string, dt_s: number, propagator?: \"j2\" }\nExample: examples/propagate.json")]
    Propagate {
        /// Path to JSON input file (see examples/propagate.json).
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Compute ROE between chief and deputy states (JSON only).
    #[command(after_help = "Input: { chief: StateVector, deputy: StateVector }\nExample: examples/roe.json")]
    Roe {
        /// Path to JSON input file (see examples/roe.json).
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Analyze safety metrics for a planned mission (JSON only).
    #[command(after_help = "Input: full PipelineInput (same as mission command).\nExample: examples/mission.json")]
    Safety {
        /// Path to JSON input file (see examples/mission.json).
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Compute eclipse intervals for a planned mission (JSON only).
    #[command(after_help = "Input: full PipelineInput (same as mission command).\nExample: examples/mission.json")]
    Eclipse {
        /// Path to JSON input file (see examples/mission.json).
        #[arg(short, long)]
        input: PathBuf,
    },

    // ---- Utilities ----
    /// Generate shell completion scripts.
    Completions {
        /// Shell to generate completions for (bash, zsh, fish, elvish, powershell).
        #[arg(value_enum)]
        shell: clap_complete::Shell,
    },
}
