use mavspec::rust::r#gen::BuildHelper; // Corrected import path for Edition 2024
use std::env;
use std::fs;
use std::path::PathBuf; // For checking directory existence

fn main() {
    //println!("cargo:warning=------------------------------------------------------------");
    //println!("cargo:warning=BUILD.RS SCRIPT STARTED");

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let definitions_dir = manifest_dir.join("mavlink_definitions");

    let sources_paths_str: Vec<&str> = vec![definitions_dir
        .to_str()
        .expect("Definitions dir path is not valid UTF-8")];

    let out_dir_path_str = env::var("OUT_DIR").unwrap();
    let out_dir = PathBuf::from(out_dir_path_str.clone()); // For checking existence
    let generation_destination = out_dir.join("mavlink_generated");

    //println!("cargo:warning=OUT_DIR is: {}", out_dir_path_str);
    //println!(
    //    "cargo:warning=Generation destination is: {}",
    //    generation_destination.display()
    //);

    // Ensure the parent `OUT_DIR` exists (Cargo should create it, but let's be sure)
    if !out_dir.exists() {
        //println!("cargo:warning=CRITICAL: OUT_DIR does not exist before generation attempt!");
        // Optionally, try to create it, though Cargo should handle this.
        // fs::create_dir_all(&out_dir).expect("Failed to create OUT_DIR");
        // println!("cargo:warning=Attempted to create OUT_DIR.");
    } else {
        //println!("cargo:warning=OUT_DIR exists as expected.");
    }

    //println!("cargo:warning=Attempting to generate MAVLink code...");
    BuildHelper::builder(&generation_destination)
        .set_sources(&sources_paths_str)
        .generate()
        .expect("BuildHelper::generate() failed inside build.rs"); // More specific expect
                                                                   //println!("cargo:warning=MAVLink code generation reported OK by BuildHelper::generate().");

    // Check if the main mod.rs was actually created
    let expected_mod_file = generation_destination.join("mod.rs");
    if expected_mod_file.exists() {
        //println!(
        //    "cargo:warning=SUCCESS: {} was created.",
        //    expected_mod_file.display()
        //);
    } else {
        //println!(
        //    "cargo:warning=FAILURE: {} was NOT created, even though generate() reported OK.",
        //    expected_mod_file.display()
        //);
        //println!(
        //    "cargo:warning=Contents of {}:",
        //    generation_destination.display()
        //);
        if generation_destination.exists() && generation_destination.is_dir() {
            for entry in fs::read_dir(&generation_destination)
                .expect("Failed to read generation_destination")
            {
                let _entry = entry.expect("Failed to read directory entry");
                //println!("cargo:warning=  - {:?}", entry.file_name());
            }
        } else {
            //println!(
            //    "cargo:warning=  (Directory {} does not exist or is not a directory)",
            //    generation_destination.display()
            //);
        }
    }
    //println!("cargo:warning=BUILD.RS SCRIPT FINISHED");
    //println!("cargo:warning=------------------------------------------------------------");
}
