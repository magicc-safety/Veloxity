use std::env;
use std::ffi::OsStr;
use std::process::{Command, ExitCode};

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let Some(command) = args.next() else {
        print_usage();
        return ExitCode::from(2);
    };

    let status = match command.as_str() {
        "check-host" => cargo(["check"]),
        "test-host" => cargo(["test", "-p", "voloxide_core"])
            .and_then(|_| cargo(["test", "-p", "sim", "--lib"])),
        "check-board" => {
            let Some(board) = args.next() else {
                eprintln!("missing board name: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            match board.as_str() {
                "nucleo" | "pixracerpro" => cargo([
                    "check",
                    "-p",
                    board.as_str(),
                    "--target",
                    "thumbv7em-none-eabihf",
                ]),
                "pico2w" => cargo([
                    "check",
                    "-p",
                    board.as_str(),
                    "--target",
                    "thumbv8m.main-none-eabihf",
                ]),
                _ => {
                    eprintln!(
                        "unknown board `{board}`: expected `nucleo`, `pixracerpro`, or `pico2w`"
                    );
                    return ExitCode::from(2);
                }
            }
        }
        "build-sim-lib" => cargo(["build", "-p", "sim", "--lib"]),
        _ => {
            eprintln!("unknown command `{command}`");
            print_usage();
            return ExitCode::from(2);
        }
    };

    match status {
        Ok(()) => ExitCode::SUCCESS,
        Err(code) => ExitCode::from(code),
    }
}

fn cargo<I, S>(args: I) -> Result<(), u8>
where
    I: IntoIterator<Item = S>,
    S: AsRef<OsStr>,
{
    let status = Command::new("cargo").args(args).status().map_err(|err| {
        eprintln!("failed to run cargo: {err}");
        1
    })?;

    if status.success() {
        Ok(())
    } else {
        Err(status.code().unwrap_or(1).try_into().unwrap_or(1))
    }
}

fn print_usage() {
    eprintln!(
        "usage: cargo xtask <command>\n\
         \n\
         commands:\n\
           check-host       check host-compatible workspace packages\n\
           test-host        run host-side Rust tests\n\
          check-board      check embedded firmware: nucleo | pixracerpro | pico2w\n\
          build-sim-lib    build the simulator static library for ROS 2"
    );
}
