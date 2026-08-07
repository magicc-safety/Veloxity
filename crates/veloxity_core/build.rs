use std::{env, fs, path::PathBuf, process::Command};

fn main() {
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    let repository_root = manifest_dir.join("../..");
    let git_dir = repository_root.join(".git");

    println!("cargo:rerun-if-env-changed=VELOXITY_GIT_HASH");
    println!("cargo:rerun-if-changed={}", git_dir.join("HEAD").display());

    let hash = env::var("VELOXITY_GIT_HASH").ok().or_else(|| {
        let output = Command::new("git")
            .args(["rev-parse", "--short=8", "HEAD"])
            .current_dir(&repository_root)
            .output()
            .ok()?;
        output.status.success().then(|| {
            String::from_utf8(output.stdout)
                .expect("git commit hash must be UTF-8")
                .trim()
                .to_owned()
        })
    });

    let hash = hash.expect(
        "Veloxity commit hash unavailable; build from a git checkout or set VELOXITY_GIT_HASH",
    );
    let version = u32::from_str_radix(hash.trim_start_matches("0x"), 16)
        .expect("VELOXITY_GIT_HASH must contain at most eight hexadecimal digits");

    if let Ok(reference) = fs::read_to_string(git_dir.join("HEAD")) {
        if let Some(reference) = reference.strip_prefix("ref: ") {
            println!(
                "cargo:rerun-if-changed={}",
                git_dir.join(reference.trim()).display()
            );
        }
    }

    let output = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("veloxity_version.rs");
    fs::write(
        output,
        format!("pub const VELOXITY_VERSION: u32 = 0x{version:08X};\n"),
    )
    .unwrap();
}
