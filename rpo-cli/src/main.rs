//! RPO mission planning CLI — porcelain and plumbing commands.

#![warn(missing_docs)]
#![warn(clippy::pedantic)]

mod cli;
mod commands;
mod error;
mod input;
mod output;

use std::process::ExitCode;

use clap::Parser;

use cli::{Cli, Command};

fn main() -> ExitCode {
    let cli = Cli::parse();

    let result = match cli.command {
        None => {
            Cli::parse_from(["rpo-cli", "--help"]);
            Ok(())
        }
        Some(Command::Mission { ref input, json }) => commands::mission::run(input, json),
        Some(Command::Validate {
            ref input,
            json,
            samples_per_leg,
            auto_drag,
        }) => commands::validate::run(input, json, samples_per_leg, auto_drag),
        Some(Command::Mc {
            ref input,
            json,
            auto_drag,
        }) => commands::mc::run(input, json, auto_drag),
        Some(Command::Classify { ref input }) => commands::classify::run(input),
        Some(Command::Transfer { ref input }) => commands::transfer::run(input),
        Some(Command::Convert { ref input, ref to }) => commands::convert::run(input, to),
        Some(Command::Propagate { ref input }) => commands::propagate::run(input),
        Some(Command::Roe { ref input }) => commands::roe::run(input),
        Some(Command::Safety { ref input }) => commands::safety::run(input),
        Some(Command::Eclipse { ref input }) => commands::eclipse::run(input),
    };

    match result {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("Error: {e}");
            e.exit_code()
        }
    }
}
