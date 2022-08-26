
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"

namespace g2o {
    std::unique_ptr<SparseOptimizer> new_sparse_optimizer();

    void create_frame_vertex(
        int vertex_id, const string& vertex_type, 
        array<double, 3> translation, array<double, 4> rotation,
        std::unique_ptr<SparseOptimizer> optimizer
        // Eigen::Matrix<double,3,1> translation , Eigen::Matrix<double,3,3> rotation
    );
} // end namespace
