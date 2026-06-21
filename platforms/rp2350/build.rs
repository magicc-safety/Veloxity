use std::{env, path::PathBuf};

fn main() {
    let manifest_dir = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").unwrap());
    println!("cargo:rustc-link-search={}", manifest_dir.display());
    println!("cargo:rerun-if-changed=memory.x");
    println!("cargo:rerun-if-env-changed=VELOXITY_IMU_TEST_ODR_HZ");
    let odr_hz = env::var("VELOXITY_IMU_TEST_ODR_HZ").unwrap_or_else(|_| "833".to_string());
    let odr_code = match odr_hz.as_str() {
        "12.5" | "12_5" | "12p5" => "125",
        "26" => "260",
        "52" => "520",
        "104" => "1040",
        "208" => "2080",
        "416" => "4160",
        "833" => "8330",
        "1666" => "16660",
        "3332" => "33320",
        "6667" => "66670",
        _ => panic!("unsupported VELOXITY_IMU_TEST_ODR_HZ={odr_hz}"),
    };
    println!("cargo:rustc-env=VELOXITY_IMU_TEST_ODR_HZ={odr_hz}");
    println!("cargo:rustc-env=VELOXITY_IMU_TEST_ODR_CODE={odr_code}");
}
