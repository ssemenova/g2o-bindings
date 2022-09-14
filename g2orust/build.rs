fn main() {
    use cmake::Config;

    cxx_build::bridge("src/lib.rs");
    let _dst = Config::new("g2o")
                    .build_target("g2o")
                    .build();

    println!("cargo:rustc-link-search=native=g2o/lib");
    println!("cargo:rustc-link-lib=static=g2o");
}
