use std::env;
use std::path::PathBuf;
use std::process::Command;

fn main() {
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let repo_root = manifest_dir
        .parent()
        .expect("the rust/ crate must live inside the d4 repository")
        .to_path_buf();

    println!("cargo:rerun-if-changed=shim/d4_shim.cpp");
    println!("cargo:rerun-if-changed=shim/d4_shim.h");
    println!("cargo:rerun-if-env-changed=D4_LIB_DIR");

    // libd4.a: honor D4_LIB_DIR, otherwise use <repo>/build, building it with
    // the repo's build.sh if it is not there yet.
    let lib_dir = env::var("D4_LIB_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| repo_root.join("build"));
    if !lib_dir.join("libd4.a").exists() {
        eprintln!("libd4.a not found in {}, running build.sh", lib_dir.display());
        let status = Command::new("bash")
            .arg("build.sh")
            .arg("-j")
            .current_dir(&repo_root)
            .status()
            .expect("failed to run build.sh");
        assert!(status.success(), "build.sh failed");
        assert!(
            lib_dir.join("libd4.a").exists(),
            "libd4.a still missing after build.sh (set D4_LIB_DIR?)"
        );
    }

    let mut build = cc::Build::new();
    build
        .cpp(true)
        .std("c++20")
        .file(manifest_dir.join("shim/d4_shim.cpp"))
        .file(repo_root.join("c++/parser/ParserDimacs.cpp"))
        .include(&repo_root)
        .include(repo_root.join("c++"))
        .include(repo_root.join("c++/parser"))
        .include(repo_root.join("3rdParty/optree-src/include"))
        // The DPLL search engine is header-template code instantiated in the
        // shim translation unit, so it must be optimized even when the Rust
        // side is built in debug mode.
        .opt_level(3)
        .define("NDEBUG", None)
        .warnings(false);
    let src_include = repo_root.join("src/include");
    if src_include.exists() {
        build.include(src_include);
    }
    build.compile("d4shim");

    println!("cargo:rustc-link-search=native={}", lib_dir.display());
    println!("cargo:rustc-link-lib=static=d4");

    // libd4.a references CaDiCaL (src/solvers wrappers) without bundling it;
    // the C++ demos obtain it through bipe. Compile the repo's bundled
    // cadical here, mirroring 3rdParty/cadical/CMakeLists.txt. Emitted after
    // the d4 link line so the linker can resolve libd4.a's references.
    let cadical_src = repo_root.join("3rdParty/cadical/src");
    let mut cadical = cc::Build::new();
    cadical
        .cpp(true)
        .opt_level(3)
        .warnings(false)
        .define("NDEBUG", None)
        .define("NBUILD", None)
        .include(&cadical_src);
    let mut cadical_cpp: Vec<PathBuf> = Vec::new();
    let mut cadical_c: Vec<PathBuf> = Vec::new();
    for entry in std::fs::read_dir(&cadical_src).expect("cannot read 3rdParty/cadical/src") {
        let path = entry.expect("cannot read dir entry").path();
        let name = path.file_name().unwrap().to_string_lossy().into_owned();
        match path.extension().and_then(|e| e.to_str()) {
            Some("cpp") if name != "cadical.cpp" && name != "mobical.cpp" => {
                cadical_cpp.push(path)
            }
            Some("c") => cadical_c.push(path),
            _ => {}
        }
    }
    cadical_cpp.sort();
    cadical_c.sort();
    cadical.files(cadical_cpp);
    cadical.compile("cadical");

    let mut cadical_cc = cc::Build::new();
    cadical_cc
        .opt_level(3)
        .warnings(false)
        .define("NDEBUG", None)
        .define("NBUILD", None)
        .include(&cadical_src)
        .files(cadical_c);
    cadical_cc.compile("cadical_c");

    if cfg!(target_os = "linux") {
        println!(
            "cargo:rustc-link-search=native={}",
            repo_root.join("3rdParty/patoh").display()
        );
        println!("cargo:rustc-link-lib=static=patoh");
    }
    println!("cargo:rustc-link-lib=gmpxx");
    println!("cargo:rustc-link-lib=gmp");
    println!("cargo:rustc-link-lib=z");
}
