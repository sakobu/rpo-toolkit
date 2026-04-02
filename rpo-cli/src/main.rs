//! RPO mission planning CLI — porcelain and plumbing commands.

#![warn(missing_docs)]
#![warn(clippy::pedantic)]

mod cli;
mod commands;
mod error;
mod input;
mod output;

use std::process::ExitCode;

use clap::{CommandFactory, Parser};
use owo_colors::set_override;

use cli::{Cli, Command, OutputMode};
use output::common::OverlayFlags;

fn main() -> ExitCode {
    let cli = Cli::parse();

    match cli.color {
        clap::ColorChoice::Always => set_override(true),
        clap::ColorChoice::Never => set_override(false),
        clap::ColorChoice::Auto => {} // owo-colors auto-detects TTY
    }

    let result = match cli.command {
        None => {
            Cli::parse_from(["rpo-cli", "--help"]);
            Ok(())
        }
        Some(Command::Mission {
            ref input,
            json,
            markdown,
            cola_threshold,
            cola_budget,
            auto_enrich,
            auto_enrich_threshold,
        }) => {
            let flags = OverlayFlags {
                cola_threshold,
                cola_budget,
                auto_enrich,
                auto_enrich_threshold,
            };
            commands::mission::run(input, OutputMode::from_flags(json, markdown), &flags)
        }
        Some(Command::Validate {
            ref input,
            json,
            markdown,
            samples_per_leg,
            auto_drag,
            cola_threshold,
            cola_budget,
            auto_enrich,
            auto_enrich_threshold,
        }) => {
            let flags = OverlayFlags {
                cola_threshold,
                cola_budget,
                auto_enrich,
                auto_enrich_threshold,
            };
            commands::validate::run(
                input,
                OutputMode::from_flags(json, markdown),
                samples_per_leg,
                auto_drag,
                &flags,
            )
        }
        Some(Command::Mc {
            ref input,
            json,
            markdown,
            auto_drag,
            cola_threshold,
            cola_budget,
            auto_enrich,
            auto_enrich_threshold,
        }) => {
            let flags = OverlayFlags {
                cola_threshold,
                cola_budget,
                auto_enrich,
                auto_enrich_threshold,
            };
            commands::mc::run(
                input,
                OutputMode::from_flags(json, markdown),
                auto_drag,
                &flags,
            )
        }
        Some(Command::Classify { ref input }) => commands::classify::run(input),
        Some(Command::Transfer { ref input }) => commands::transfer::run(input),
        Some(Command::Convert { ref input, ref to }) => commands::convert::run(input, to),
        Some(Command::Propagate { ref input }) => commands::propagate::run(input),
        Some(Command::Roe { ref input }) => commands::roe::run(input),
        Some(Command::Safety { ref input }) => commands::safety::run(input),
        Some(Command::Eclipse { ref input }) => commands::eclipse::run(input),
        Some(Command::Completions { shell }) => {
            let mut cmd = Cli::command();
            clap_complete::generate(shell, &mut cmd, "rpo-cli", &mut std::io::stdout());
            Ok(())
        }
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e}");
            e.exit_code()
        }
    }
}
