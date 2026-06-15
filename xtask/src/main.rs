use std::env;
use std::ffi::OsStr;
use std::fs;
use std::path::Path;
use std::process::{Command, ExitCode};

fn main() -> ExitCode {
    let mut args = env::args().skip(1);
    let Some(command) = args.next() else {
        print_usage();
        return ExitCode::from(2);
    };

    let status = match command.as_str() {
        "check-host" => cargo(["check"]),
        "test-host" => cargo(["test", "-p", "veloxity_core"])
            .and_then(|_| cargo(["test", "-p", "sim", "--lib"])),
        "check-board" => {
            let Some(board) = args.next() else {
                eprintln!("missing board name: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            let Some(target) = board_target(&board) else {
                eprintln!("unknown board `{board}`: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            cargo([
                "check",
                "-p",
                board.as_str(),
                "--target",
                target,
                "--bin",
                "veloxity",
            ])
        }
        "build-board" => {
            let Some(board) = args.next() else {
                eprintln!("missing board name: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            let Some(target) = board_target(&board) else {
                eprintln!("unknown board `{board}`: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            cargo([
                "build",
                "-p",
                board.as_str(),
                "--target",
                target,
                "--bin",
                "veloxity",
            ])
        }
        "flash-board" => {
            let Some(board) = args.next() else {
                eprintln!("missing board name: expected `pico2w`");
                return ExitCode::from(2);
            };
            let Some(target) = board_target(&board) else {
                eprintln!("unknown board `{board}`: expected `nucleo`, `pixracerpro`, or `pico2w`");
                return ExitCode::from(2);
            };
            cargo([
                "run",
                "-p",
                board.as_str(),
                "--target",
                target,
                "--bin",
                "veloxity",
            ])
        }
        "build-sim-lib" => cargo(["build", "-p", "sim", "--lib"]),
        "clean-generated" => clean_generated(),
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

fn clean_generated() -> Result<(), u8> {
    for path in [
        "target",
        "workspace",
        "rosflight_memory",
        "tools/__pycache__",
    ] {
        remove_dir_if_exists(path)?;
    }

    let espnow_dir = Path::new("tools/espnow_uart_bridge");
    if espnow_dir.exists() {
        for entry in fs::read_dir(espnow_dir).map_err(|err| {
            eprintln!("failed to read {}: {err}", espnow_dir.display());
            1
        })? {
            let entry = entry.map_err(|err| {
                eprintln!("failed to read ESP-NOW bridge directory entry: {err}");
                1
            })?;
            let file_name = entry.file_name();
            let file_name = file_name.to_string_lossy();
            if file_name.starts_with("build") && entry.path().is_dir() {
                remove_dir_if_exists(entry.path())?;
            }
        }

        remove_file_if_exists(espnow_dir.join("sdkconfig"))?;
        remove_file_if_exists(espnow_dir.join("dependencies.lock"))?;
    }

    Ok(())
}

fn board_target(board: &str) -> Option<&'static str> {
    match board {
        "nucleo" | "pixracerpro" => Some("thumbv7em-none-eabihf"),
        "pico2w" => Some("thumbv8m.main-none-eabihf"),
        _ => None,
    }
}

fn remove_dir_if_exists<P>(path: P) -> Result<(), u8>
where
    P: AsRef<Path>,
{
    let path = path.as_ref();
    if !path.exists() {
        return Ok(());
    }
    if !path.is_dir() {
        eprintln!("refusing to remove non-directory {}", path.display());
        return Err(1);
    }

    println!("removing {}", path.display());
    fs::remove_dir_all(path).map_err(|err| {
        eprintln!("failed to remove {}: {err}", path.display());
        1
    })
}

fn remove_file_if_exists<P>(path: P) -> Result<(), u8>
where
    P: AsRef<Path>,
{
    let path = path.as_ref();
    if !path.exists() {
        return Ok(());
    }
    if !path.is_file() {
        eprintln!("refusing to remove non-file {}", path.display());
        return Err(1);
    }

    println!("removing {}", path.display());
    fs::remove_file(path).map_err(|err| {
        eprintln!("failed to remove {}: {err}", path.display());
        1
    })
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
           build-board      build embedded firmware: nucleo | pixracerpro | pico2w\n\
           flash-board      build and flash embedded firmware with probe-rs\n\
           build-sim-lib    build the simulator static library for ROS 2\n\
           clean-generated  remove ignored local build/runtime artifacts"
    );
}
