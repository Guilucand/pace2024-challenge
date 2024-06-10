#[cfg(feature = "build-headers")]
extern crate bindgen;

use flate2::read::GzDecoder;
use std::env;
use std::error::Error;
use std::fs::File;
use std::path::PathBuf;
use tar::Archive;
use xz::read::XzDecoder;

#[cfg(feature = "build-headers")]
fn _build_headers_from_scip_dir(path: &str) -> bindgen::Builder {
    let lib_dir = PathBuf::from(&path).join("lib");
    let lib_dir_path = lib_dir.to_str().unwrap();

    if lib_dir.exists() {
        println!("cargo:warning=Using SCIP from {}", lib_dir_path);
        println!("cargo:rustc-link-search={}", lib_dir_path);
        println!("cargo:libdir={}", lib_dir_path);

        #[cfg(windows)]
        let lib_dir_path = PathBuf::from(&path).join("bin");
        #[cfg(windows)]
        println!("cargo:rustc-link-search={}", lib_dir_path.to_str().unwrap());
    } else {
        panic!(
            "{}",
            format!(
                "{}/lib does not exist, please check your SCIP installation",
                path
            )
        );
    }

    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", lib_dir_path);

    let include_dir = PathBuf::from(&path).join("include");
    let include_dir_path = include_dir.to_str().unwrap();
    let scip_header_file = PathBuf::from(&path)
        .join("include")
        .join("scip")
        .join("scip.h")
        .to_str()
        .unwrap()
        .to_owned();
    let scipdefplugins_header_file = PathBuf::from(&path)
        .join("include")
        .join("scip")
        .join("scipdefplugins.h")
        .to_str()
        .unwrap()
        .to_owned();

    bindgen::Builder::default()
        .header(scip_header_file)
        .header(scipdefplugins_header_file)
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .clang_arg(format!("-I{}", include_dir_path))
}

fn main() -> Result<(), Box<dyn Error>> {
    let crate_root = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    let lib_path = out_path.join("lib");
    let build_libs = if lib_path.exists() {
        println!("cargo:warning=SCIP was previously compiled, skipping compilation");
        false
    } else {
        true
    };

    // Build gmp
    let gmp_build_path = {
        let source_archive = crate_root.join("sources/gmp-6.3.0.tar.xz");
        println!(
            "cargo:warning=GMP source archive: {}",
            source_archive.display()
        );
        let build_path = out_path.join("build_deps/gmp");

        if build_libs {
            let tar_gz = File::open(source_archive)?;
            let tar = XzDecoder::new(tar_gz);
            let mut archive = Archive::new(tar);
            archive.unpack(&out_path)?;

            let source_dir = out_path.join("gmp-6.3.0");
            let _ = std::fs::create_dir_all(&build_path);

            let _ = autotools::Config::new(source_dir)
                .config_option("prefix", Some(&format!("{}", build_path.display())))
                .config_option("enable-static", Some("yes"))
                .config_option("enable-shared", Some("no"))
                .with("pic", None)
                .build();
        }
        build_path
    };

    // Build mpfr
    let mpfr_build_path = {
        let source_archive = crate_root.join("sources/mpfr-4.2.1.tar.gz");
        println!(
            "cargo:warning=MPFR source archive: {}",
            source_archive.display()
        );
        let build_path = out_path.join("build_deps/mpfr");

        if build_libs {
            let tar_gz = File::open(source_archive)?;
            let tar = GzDecoder::new(tar_gz);
            let mut archive = Archive::new(tar);
            archive.unpack(&out_path)?;

            let source_dir = out_path.join("mpfr-4.2.1");
            let _ = std::fs::create_dir_all(&build_path);

            let _ = autotools::Config::new(source_dir)
                .config_option("prefix", Some(&format!("{}", build_path.display())))
                .config_option("enable-static", Some("yes"))
                .config_option("disable-shared", None)
                .with("pic", None)
                .with("gmp", Some(&format!("{}", gmp_build_path.display())))
                .config_option("enable-cxx", None)
                .build();
        }
        build_path
    };

    // Build SCIP
    let scip_build_path = {
        if build_libs {
            let source_zip = crate_root.join("sources/scipoptsuite-9.0.0.zip");
            println!("cargo:warning=SCIP source zip: {}", source_zip.display());

            zip_extract::extract(
                std::io::Cursor::new(std::fs::read(source_zip).unwrap()),
                &out_path,
                false,
            )?;
            let source_dir = out_path.join("scipoptsuite-9.0.0");

            use cmake::Config;
            let mut dst = Config::new(source_dir);
            dst.define("AUTOBUILD", "OFF")
                .define("SHARED", "OFF")
                .define("ZLIB", "OFF")
                .define("READLINE", "OFF")
                .define("GMP", "ON")
                .define("STATIC_GMP", "OFF")
                .define("PAPILO", "OFF")
                .define("ZIMPL", "OFF")
                .define("AMPL", "OFF")
                .define("IPOPT", "OFF")
                .define("GMP_DIR", &format!("{}", gmp_build_path.display()))
                .define("CMAKE_CXX_FLAGS", "-DHZ=100 -fPIE")
                .define("CMAKE_C_FLAGS", "-DHZ=100 -fPIE")
                .define("IPOPT", "OFF")
                .target("libscip")
                .build()
        } else {
            out_path.clone()
        }
    };

    println!(
        "cargo:warning=Finished building dependencies, SCIP path: {}",
        scip_build_path.display()
    );

    #[cfg(feature = "build-headers")]
    {
        let builder = _build_headers_from_scip_dir(&scip_build_path.to_str().unwrap());
        let builder = builder
            .blocklist_item("FP_NAN")
            .blocklist_item("FP_INFINITE")
            .blocklist_item("FP_ZERO")
            .blocklist_item("FP_SUBNORMAL")
            .blocklist_item("FP_NORMAL")
            .parse_callbacks(Box::new(bindgen::CargoCallbacks));

        let bindings = builder.generate()?;
        let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
        bindings.write_to_file(out_path.join("bindings.rs"))?;
    }

    #[cfg(not(feature = "build-headers"))]
    {
        println!("cargo:warning=Skipping header generation");
        std::fs::copy(
            crate_root.join("sources/bindings.rs"),
            out_path.join("bindings.rs"),
        )?;
    }

    println!("cargo:rustc-link-search={}/lib", gmp_build_path.display());
    println!("cargo:rustc-link-search={}/lib", mpfr_build_path.display());
    println!("cargo:rustc-link-search={}/lib", scip_build_path.display());
    println!("cargo:rustc-link-search={}/build/lib", out_path.display());

    println!("cargo:rustc-link-lib=scip");
    println!("cargo:rustc-link-lib=soplex");
    println!("cargo:rustc-link-lib=bliss");
    println!("cargo:rustc-link-lib=mpfr");
    println!("cargo:rustc-link-lib=gmp");
    println!("cargo:rustc-link-lib=stdc++");

    // let bindings = builder.generate()?;

    Ok(())
}
