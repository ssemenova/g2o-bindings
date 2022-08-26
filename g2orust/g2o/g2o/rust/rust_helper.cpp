
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../types/se3quat.h"
#include "../types/types_six_dof_expmap.h"

namespace g2o {
    std::unique_ptr<SparseOptimizer> new_sparse_optimizer() {
        std::unique_ptr<SparseOptimizer> optimizer = std::make_unique<SparseOptimizer>();

        BlockSolver_6_3::LinearSolverType * linearSolver;
        linearSolver = new LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();

        BlockSolver_6_3* solver_ptr = new BlockSolver_6_3(linearSolver);
        OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer->setAlgorithm(solver);

        return optimizer;
    }

    void create_frame_vertex(
        int vertex_id, const string& vertex_type, 
        array<double, 3> translation, array<double, 4> rotation,
        std::unique_ptr<SparseOptimizer> optimizer
    ) {
        if (vertex_type == "VertexSE3Expmap") {
            VertexSE3Expmap * vSE3 = new VertexSE3Expmap();

            Eigen::Vector3d trans_vec(translation.data());
            Eigen::Quaterniond rot_quat(rotation.data());
            // Rotation is already a quaternion!!
            vSE3->setEstimate(SE3Quat(rot_quat, trans_vec));

            vSE3->setId(0);
            vSE3->setFixed(false);
            optimizer->addVertex(vSE3);
        }
    }

    void add_edge_monocular(
        std::unique_ptr<SparseOptimizer> optimizer
    ) {
        Eigen::Matrix<double,2,1> obs;
        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
        obs << kpUn.pt.x, kpUn.pt.y;

        EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

        e->setVertex(
            0, 
            dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0))
        );
        e->setMeasurement(obs);
        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
        e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(deltaMono);

        e->pCamera = pFrame->mpCamera;
        e->Xw = pMP->GetWorldPos().cast<double>();

        optimizer.addEdge(e);

        vpEdgesMono.push_back(e);
        vnIndexEdgeMono.push_back(i);

    }

    void 

    void _add_edge_stereo() {
        // Eigen::Matrix<double,3,1> obs;
        // const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
        // const float &kp_ur = pFrame->mvuRight[i];
        // obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

        // g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

        // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(0)));
        // e->setMeasurement(obs);
        // const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
        // Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
        // e->setInformation(Info);

        // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        // e->setRobustKernel(rk);
        // rk->setDelta(deltaStereo);

        // e->fx = pFrame->fx;
        // e->fy = pFrame->fy;
        // e->cx = pFrame->cx;
        // e->cy = pFrame->cy;
        // e->bf = pFrame->mbf;
        // e->Xw = pMP->GetWorldPos().cast<double>();

        // optimizer.addEdge(e);

        // vpEdgesStereo.push_back(e);
        // vnIndexEdgeStereo.push_back(i);
    }


} // end namespace
