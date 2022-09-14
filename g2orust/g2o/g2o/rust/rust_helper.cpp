
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../types/se3quat.h"
#include "../types/types_six_dof_expmap.h"
#include "rust_helper.h"

namespace g2o {
    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer() {
        return std::unique_ptr<BridgeSparseOptimizer>(new BridgeSparseOptimizer());
    }

    BridgeSparseOptimizer::BridgeSparseOptimizer() {
        // ORB_SLAM3 Optimizer::PoseOptimization lines 817-825
        optimizer = std::make_unique<SparseOptimizer>();
        linearSolver = new LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();
        solver_ptr = new BlockSolver_6_3(linearSolver);
        solver = new OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer->setAlgorithm(solver);

        deltaMono = sqrt(5.991);
        deltaStereo = sqrt(7.815);
    }

    BridgeSparseOptimizer::~BridgeSparseOptimizer() {
        delete linearSolver;
        delete solver_ptr;
        delete solver;
    }
 
    void BridgeSparseOptimizer::create_frame_vertex (
        int vertex_id, const string& vertex_type, 
        array<double, 3> translation, array<double, 4> rotation
    ) const {

        // ORB_SLAM3 Optimizer::PoseOptimization lines 830-835
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

    void BridgeSparseOptimizer::add_edge_monocular(
        int index,
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y,
        float invSigma2
    ) const {
        // ORB_SLAM3 Optimizer::PoseOptimization lines 869-893

        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        EdgeSE3ProjectXYZOnlyPose* e = new EdgeSE3ProjectXYZOnlyPose();

        e->setVertex(
            0, 
            dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(0))
        );
        // e->setMeasurement(obs);
        // // const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
        // e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        // RobustKernelHuber* rk = new RobustKernelHuber;
        // e->setRobustKernel(rk);
        // rk->setDelta(deltaMono);

        // e->pCamera = pFrame->mpCamera;
        // e->Xw = pMP->GetWorldPos().cast<double>();

        optimizer->addEdge(e);

        // vpEdgesMono.push_back(e);
        // vnIndexEdgeMono.push_back(index);

        // Sofiya...not copied here is lines 931-996
        // "SLAM with respect to a rigid body"
        // Is alternative to add_edge_monocular and add_edge_stereo
        // but not sure why it would be used
    }

    void BridgeSparseOptimizer::_add_edge_stereo() const {
        // TODO (Stereo)
        // ORB_SLAM3 Optimizer::PoseOptimization lines 897-928

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

    // void BridgeSparseOptimizer::optimize() {
    //     // ORB_SLAM3 Optimizer::PoseOptimization lines 1004-1109

    //     // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    //     // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    //     const float chi2Mono[4]={5.991,5.991,5.991,5.991};
    //     const float chi2Stereo[4]={7.815,7.815,7.815, 7.815};
    //     const int its[4]={10,10,10,10};    

    //     int nBad=0;
    //     for(size_t it=0; it<4; it++) {
    //         Tcw = pFrame->GetPose();
    //         vSE3->setEstimate(g2o::SE3Quat(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>()));

    //         optimizer.initializeOptimization(0);
    //         optimizer.optimize(its[it]);

    //         nBad=0;
    //         for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++) {
    //             ORB_SLAM3::EdgeSE3ProjectXYZOnlyPose* e = vpEdgesMono[i];

    //             const size_t idx = vnIndexEdgeMono[i];

    //             if(pFrame->mvbOutlier[idx])
    //             {
    //                 e->computeError();
    //             }

    //             const float chi2 = e->chi2();

    //             if(chi2>chi2Mono[it])
    //             {                
    //                 pFrame->mvbOutlier[idx]=true;
    //                 e->setLevel(1);
    //                 nBad++;
    //             }
    //             else
    //             {
    //                 pFrame->mvbOutlier[idx]=false;
    //                 e->setLevel(0);
    //             }

    //             if(it==2)
    //                 e->setRobustKernel(0);
    //         }

    //         for(size_t i=0, iend=vpEdgesMono_FHR.size(); i<iend; i++) {
    //             ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

    //             const size_t idx = vnIndexEdgeRight[i];

    //             if(pFrame->mvbOutlier[idx]) {
    //                 e->computeError();
    //             }

    //             const float chi2 = e->chi2();

    //             if(chi2>chi2Mono[it]) {
    //                 pFrame->mvbOutlier[idx]=true;
    //                 e->setLevel(1);
    //                 nBad++;
    //             }
    //             else {
    //                 pFrame->mvbOutlier[idx]=false;
    //                 e->setLevel(0);
    //             }

    //             if(it==2)
    //                 e->setRobustKernel(0);
    //         }

    //         for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++) {
    //             g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

    //             const size_t idx = vnIndexEdgeStereo[i];

    //             if(pFrame->mvbOutlier[idx]) {
    //                 e->computeError();
    //             }

    //             const float chi2 = e->chi2();

    //             if(chi2>chi2Stereo[it]) {
    //                 pFrame->mvbOutlier[idx]=true;
    //                 e->setLevel(1);
    //                 nBad++;
    //             }
    //             else {
    //                 e->setLevel(0);
    //                 pFrame->mvbOutlier[idx]=false;
    //             }

    //             if(it==2)
    //                 e->setRobustKernel(0);
    //         }
    //         if(optimizer.edges().size()<10)
    //             break;
    //     }

    //     // Recover optimized pose and return number of inliers
    //     g2o::VertexSE3Expmap* vSE3_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    //     g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
    //     Sophus::SE3<float> pose(SE3quat_recov.rotation().cast<float>(),
    //             SE3quat_recov.translation().cast<float>());
    //     pFrame->SetPose(pose);

    //     return nInitialCorrespondences-nBad;
    // }

} // end namespace
