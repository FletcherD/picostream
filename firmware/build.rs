use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

    // Select memory.x based on target chip
    let memory_x = if env::var("CARGO_FEATURE_RP235X").is_ok() {
        include_bytes!("memory-rp235x.x").as_slice()
    } else {
        include_bytes!("memory-rp2040.x").as_slice()
    };

    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(memory_x)
        .unwrap();

    println!("cargo:rustc-link-search={}", out.display());
    println!("cargo:rerun-if-changed=memory-rp2040.x");
    println!("cargo:rerun-if-changed=memory-rp235x.x");
    println!("cargo:rerun-if-changed=build.rs");

    // Linker arguments
    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");

    // RP2040 needs link-rp.x, RP235x does not
    if env::var("CARGO_FEATURE_RP2040").is_ok() {
        println!("cargo:rustc-link-arg-bins=-Tlink-rp.x");
    }

    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
