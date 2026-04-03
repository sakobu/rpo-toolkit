//! CLI definition: clap parser and command enum.
//!
//! Porcelain commands (Mission, Validate, MC) share identical flag sets for
//! output mode, COLA, and formation enrichment via [`PorcelainArgs`] with
//! `#[command(flatten)]`. Command-specific flags (e.g., `--auto-drag`,
//! `--samples-per-leg`) live on the individual variants.

use std::path::PathBuf;

use clap::{Args, Parser, Subcommand};

use crate::output::common::OverlayFlags;

/// Output mode for porcelain commands.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    /// Markdown summary. Prints to stdout unless `-o` is given.
    Summary,
    /// Machine-readable JSON. Prints to stdout unless `-o` is given.
    Json,
}

/// Shared flags for all porcelain commands.
///
/// Groups output mode, file destination, COLA, and formation enrichment
/// flags to avoid duplication across Mission / Validate / MC variants.
#[derive(Args, Debug)]
pub struct PorcelainArgs {
    /// Output as JSON instead of markdown summary.
    #[arg(long)]
    pub json: bool,
    /// Write output to file instead of stdout.
    #[arg(short, long, value_name = "PATH")]
    pub output: Option<PathBuf>,
    /// Target miss distance for collision avoidance (km). Enables COLA.
    #[arg(long, value_name = "KM")]
    pub cola_threshold: Option<f64>,
    /// Maximum delta-v budget for collision avoidance (km/s).
    #[arg(long, value_name = "KM_S", requires = "cola_threshold")]
    pub cola_budget: Option<f64>,
    /// Enable formation design enrichment with default 0.1 km separation threshold.
    /// Enriches perch ROE with safe e/i vectors (D'Amico Eq. 2.22) and computes
    /// per-leg transit safety profiles.
    #[arg(long)]
    pub auto_enrich: bool,
    /// Custom min separation threshold (km) for formation design enrichment.
    /// Implies `--auto-enrich`. Overrides the default 0.1 km threshold.
    #[arg(long, value_name = "KM")]
    pub auto_enrich_threshold: Option<f64>,
}

impl PorcelainArgs {
    /// Resolve output mode from the `--json` flag.
    #[must_use]
    pub fn mode(&self) -> OutputMode {
        if self.json {
            OutputMode::Json
        } else {
            OutputMode::Summary
        }
    }

    /// Build overlay flags from COLA and enrichment CLI flags.
    #[must_use]
    pub fn overlay_flags(&self) -> OverlayFlags {
        OverlayFlags {
            cola_threshold: self.cola_threshold,
            cola_budget: self.cola_budget,
            auto_enrich: self.auto_enrich,
            auto_enrich_threshold: self.auto_enrich_threshold,
        }
    }
}

/// RPO mission planning tool.
#[derive(Parser)]
#[command(
    name = "rpo-cli",
    about = "RPO mission planning tool",
    after_help = "Porcelain commands produce a markdown summary by default (use --json for machine output).\n\
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
        /// Shared porcelain flags (output mode, COLA, enrichment).
        #[command(flatten)]
        args: PorcelainArgs,
    },
    /// End-to-end mission with Nyx high-fidelity validation (requires network on first run).
    Validate {
        /// Path to JSON input file with chief/deputy states, perch, waypoints, and spacecraft configs.
        #[arg(short, long)]
        input: PathBuf,
        /// Shared porcelain flags (output mode, COLA, enrichment).
        #[command(flatten)]
        args: PorcelainArgs,
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
        /// Shared porcelain flags (output mode, COLA, enrichment).
        #[command(flatten)]
        args: PorcelainArgs,
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
