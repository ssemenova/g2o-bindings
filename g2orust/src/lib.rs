use nalgebra::{Isometry3};

#[cxx::bridge(namespace = "g2o")]
pub mod ffi {
    // C++ types and signatures exposed to Rust.
    unsafe extern "C++" {
        include!("/home/sofiya/g2o-bindings/g2orust/g2o/g2o/rust/rust_helper.h");

        // Zero or more opaque types which both languages can pass around
        // but only C++ can see the fields.
        type BridgeSparseOptimizer;

        fn new_sparse_optimizer() -> UniquePtr<BridgeSparseOptimizer>;
        fn create_frame_vertex(
            &self,
            vertex_id: i32, vertex_type: &CxxString, 
            translation: [f64; 3], rotation: [f64; 4]
        );
        fn add_edge_monocular(
            &self,
            index: i32,
            keypoint_octave: i32,
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            invSigma2: f32,
        );
        fn _add_edge_stereo(&self);

    }
}