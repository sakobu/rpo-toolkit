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

use cli::{Cli, Command};

fn main() -> ExitCode {
    let cli = Cli::parse();

    let result = match cli.command {
        None => {
            Cli::parse_from(["rpo-cli", "--help"]);
            Ok(())
        }
        Some(Command::Mission { ref input, ref args }) => {
            commands::mission::run(input, args.mode(), args.output.as_deref(), &args.overlay_flags())
        }
        Some(Command::Validate {
            ref input,
            ref args,
            samples_per_leg,
            auto_drag,
        }) => commands::validate::run(
            input,
            args.mode(),
            args.output.as_deref(),
            samples_per_leg,
            auto_drag,
            &args.overlay_flags(),
        ),
        Some(Command::Mc {
            ref input,
            ref args,
            auto_drag,
        }) => commands::mc::run(
            input,
            args.mode(),
            args.output.as_deref(),
            auto_drag,
            &args.overlay_flags(),
        ),
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
