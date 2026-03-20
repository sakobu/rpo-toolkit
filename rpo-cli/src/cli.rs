//! CLI definition: clap parser and command enum.

use std::path::PathBuf;

use clap::{Parser, Subcommand};

/// RPO mission planning tool.
#[derive(Parser)]
#[command(
    name = "rpo-cli",
    about = "RPO mission planning tool",
    after_help = "Porcelain commands produce human-readable output by default (use --json for machine output).\n\
                  Plumbing commands always produce JSON."
)]
pub struct Cli {
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
    /// End-to-end mission with nyx high-fidelity validation (requires network on first run).
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
        /// Auto-derive differential drag config from spacecraft properties via nyx.
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
        /// Auto-derive differential drag config from spacecraft properties via nyx.
        #[arg(long)]
        auto_drag: bool,
    },

    // ---- Plumbing ----
    /// Classify chief/deputy separation (JSON only).
    Classify {
        /// Path to JSON input file.
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Solve Lambert transfer (JSON only).
    Transfer {
        /// Path to JSON input file.
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Convert between state representations (JSON only).
    Convert {
        /// Path to JSON input file (ECI state).
        #[arg(short, long)]
        input: PathBuf,
        /// Target representation: keplerian, roe.
        #[arg(long, value_name = "FORMAT")]
        to: String,
    },
    /// Propagate ROE state via STM (JSON only).
    Propagate {
        /// Path to JSON input file.
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Compute ROE between chief and deputy states (JSON only).
    Roe {
        /// Path to JSON input file.
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Analyze safety metrics for a planned mission (JSON only).
    Safety {
        /// Path to JSON input file.
        #[arg(short, long)]
        input: PathBuf,
    },
    /// Compute eclipse intervals for a planned mission (JSON only).
    Eclipse {
        /// Path to JSON input file.
        #[arg(short, long)]
        input: PathBuf,
    },
}
