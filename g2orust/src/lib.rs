use nalgebra::{Isometry3};

#[cxx::bridge(namespace = "g2o")]
pub mod ffi {
    // Shared structs with fields visible to both languages.
    struct BlobMetadata {
        size: usize,x
        tags: Vec<String>,
    }

    // Rust types and signatures exposed to C++.
    // extern "Rust" {
    //     type MultiBuf;

    //     fn next_chunk(buf: &mut MultiBuf) -> &[u8];
    // }

    // C++ types and signatures exposed to Rust.
    unsafe extern "C++" {
        include!("/home/sofiya/g2o-bindings/g2orust/g2o/g2o/rust/rust_helper.h");

        // Zero or more opaque types which both languages can pass around
        // but only C++ can see the fields.
        type SparseOptimizer;

        fn new_sparse_optimizer() -> UniquePtr<SparseOptimizer>;
        fn create_frame_vertex(
            vertex_id: i32, vertex_type: &CxxString, 
            translation: [f64; 3], rotation: [f64; 4],
            optimizer: UniquePtr<SparseOptimizer>);
        // fn put(&self, parts: &mut MultiBuf) -> u64;
        // fn tag(&self, blobid: u64, tag: &str);
        // fn metadata(&self, blobid: u64) -> BlobMetadata;
    }
}