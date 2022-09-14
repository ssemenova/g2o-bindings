
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../types/types_six_dof_expmap.h"

namespace g2o {

    class BridgeSparseOptimizer {
    public:
        BridgeSparseOptimizer();
        ~BridgeSparseOptimizer();
        void create_frame_vertex (
            int vertex_id, const string& vertex_type, 
            array<double, 3> translation, array<double, 4> rotation
        ) const;
        void add_edge_monocular(
            int index,
            int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y,
            float invSigma2
        ) const;
        void _add_edge_stereo() const;

    private:
        std::unique_ptr<SparseOptimizer> optimizer;
        BlockSolver_6_3::LinearSolverType * linearSolver;
        BlockSolver_6_3* solver_ptr;
        OptimizationAlgorithmLevenberg* solver;
        float deltaMono;
        float deltaStereo;
        // vpEdgesMono;
        vector<std::unique_ptr<EdgeSE3ProjectXYZOnlyPose>> vpEdgesMono;
        vector<size_t> vnIndexEdgeMono;
    };

    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer();
} // end namespace
