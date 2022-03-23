/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */

#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include <Eigen/StdVector>

#include "Converter.h"

#include <mutex>

namespace ORB_SLAM2
{

    void Optimizer::GlobalBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        // Optimizer的全局BA函数，传入的参数是建立的地图和迭代次数
        // 首先调用Map类的成员函数GetAllKeyFrames返回与当前Map有关联的所有关键帧
        vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        // 然后调用Map类的成员函数GetAllMapPoints返回当前地图中的所有关键点
        vector<MapPoint *> vpMP = pMap->GetAllMapPoints();
        // 调用BA函数准备优化，未指定的参数在声明中都有默认值(Optimizer.h中函数定义时定义了一些传入参数的默认值)
        BundleAdjustment(vpKFs, vpMP, nIterations, pbStopFlag, nLoopKF, bRobust);
    }

    void Optimizer::BundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                     int nIterations, bool *pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    {
        // 实现BA的核心函数
        vector<bool> vbNotIncludedMP;
        vbNotIncludedMP.resize(vpMP.size());

        // 调用的是g2o的优化API，g2o使用的固定格式
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
        // 设置优化算法是Levenberg
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // 是否设置强制停止标志，默认为NULL
        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        long unsigned int maxKFid = 0;

        // Set KeyFrame vertices
        // 向优化图中增加顶点，这里的顶点就是与当前地图相关联的关键帧
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            // 依次获取关键帧对象
            KeyFrame *pKF = vpKFs[i];
            // 如果当前关键帧是坏的，则跳过
            if (pKF->isBad())
                continue;
            // 新建出来一个SE3指数映射的一个对象指针vSE3，它其实可以理解为关键帧的位姿，对应一个三维欧式变换矩阵T
            // SE3中文叫做特殊欧式群，是一个由R、t组成的4*4的矩阵
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            // 将我们当前的估值传入节点
            // 这里调用了一个将SE3转换为四元数（表示旋转）和t（表示平移）的函数
            // 首先通过KeyFrame成员函数GetPose获取了以Mat表示的当前位姿
            // 然后用Convert中的函数，将Mat转换成g2o中的SE3Quat对象，包含R、t
            vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
            // 将当前关键帧ID也赋给节点
            vSE3->setId(pKF->mnId);
            // 该节点是否固定，如果固定，则在优化中就不会调整它的位姿，反之就会调整
            // 这里的做法是只对ID等于0的关键帧（初始帧）设置为固定，对于其它所有帧都不固定，优化时都可以改变位置；
            // mnId是关键帧的ID
            vSE3->setFixed(pKF->mnId == 0);
            // 最后一步，向优化器中添加节点
            optimizer.addVertex(vSE3);
            // 如果关键帧ID大于目前的最大ID，就进行替换
            if (pKF->mnId > maxKFid)
                maxKFid = pKF->mnId;
        }

        // 设置了两个阈值，分别针对后面单目和双目两种不同情况
        // 2D：单目；3D：双目
        const float thHuber2D = sqrt(5.99);
        const float thHuber3D = sqrt(7.815);

        // Set MapPoint vertices
        // 向优化图中添加地图节点和对应边
        // 下面的代码是两个非常长的for循环的嵌套
        // 简单来说就是从地图点的角度，外层循环遍历每个地图点
        // 对于每个地图点，由于可能有多个对应的观测（关键帧），所以内层再用一个循环遍历每个关键帧
        // 这样就可以将与某个地图点相关联的所有边添加到优化图中了
        // 重复外层循环，就可以将所有点的所有关联边添加到优化图中了
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            // 首先，遍历vector中的每一个地图点
            MapPoint *pMP = vpMP[i];
            // 如果地图点是坏的，本次循环结束，不添加这个地图点进优化图
            if (pMP->isBad())
                continue;
            // 这里也是新建了节点用于存放数据，需要注意的是这里的节点类型和上面的是不一样的
            // 上面的节点存放的是位姿信息，所以节点类型是VertexSE3Expmap
            // 而这里的节点存放的就是地图点的(x,y,z)坐标，所以类型是VertexSBAPointXYZ
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            // 新建好对象指针后，还是老套路，调用它的成员函数setEstimate来设置当前我们计算的数据
            // 然后调用Converter的转换函数，将其转换成Eigen的Matrix类型
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            // 计算该地图点的ID，计算公式是当前地图点的ID加上关键帧的最大ID再加1
            // 这样做的原因是避免和前面关键帧的ID重复，关键帧ID先排完，然后是地图点
            const int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            // 设置是否边缘化，对于这个函数，文档解释是set this node should be marginalized out during the optimization
            // 在优化是，该节点是否被边缘化
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);    // 最后一步，添加地图点到优化图中

            // 上面经过了两大步骤，分别向优化图中添加了关键帧节点和地图点节点
            // 所以下面理应所当地应该添加各个节点之间的联系了
            // 这种关系就是之前代码中提到的“观测”，这种观测信息存放在地图点中，从地图点的角度描述的；某个地图点能被哪几个关键帧观测到

            // 首先获取当前地图点观测
            // 关键帧作为键(key)，值(value)是该地图点在该关键帧特征点列表中的索引
            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            int nEdges = 0;
            // SET EDGES 依次遍历每个地图点，对每个地图点的观测添加边
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++)
            {
                // 首先获取该地图点对应的关键帧
                KeyFrame *pKF = mit->first;
                // 如果这个关键帧是坏的或者索引超过了最大索引，就跳过这个关键帧，不添加了
                if (pKF->isBad() || pKF->mnId > maxKFid)
                    continue;

                // 否则边的个数加1
                nEdges++;

                // 根据上面说的，在这里定义的观测是一个键值对，键为关键帧，值为这个地图点在这个关键帧特征点列表中的索引
                // 所以，这里先获取到了这个地图点在关键帧中的索引，然后通过索引获取到这个关键帧成员变量mvKeysUn对应的元素
                // 这个对象就是在关键帧中的二维特征点
                const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

                // 这里针对不同情况进行了一个判断，即mvuRight中对应位置的元素值是否小于0
                // 它最一开始是在Frame.cc的228行赋值的，关键帧KeyFrame中的mvuRight是在构造函数中直接拷贝的对应Frame的mvuRight(KeyFrame.cc,41行)
                // 它主要是针对双目情况，对于单目情况，它的所有的元素都为-1(Frame.cc,228行)
                
                if (pKF->mvuRight[mit->second] < 0)
                {
                    Eigen::Matrix<double, 2, 1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber2D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;

                    optimizer.addEdge(e);
                }
                else
                {
                    Eigen::Matrix<double, 3, 1> obs;
                    const float kp_ur = pKF->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                    e->setInformation(Info);

                    if (bRobust)
                    {
                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuber3D);
                    }

                    e->fx = pKF->fx;
                    e->fy = pKF->fy;
                    e->cx = pKF->cx;
                    e->cy = pKF->cy;
                    e->bf = pKF->mbf;

                    optimizer.addEdge(e);
                }
            }

            if (nEdges == 0)
            {
                optimizer.removeVertex(vPoint);
                vbNotIncludedMP[i] = true;
            }
            else
            {
                vbNotIncludedMP[i] = false;
            }
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(nIterations);

        // Recover optimized data

        // Keyframes
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            if (nLoopKF == 0)
            {
                pKF->SetPose(Converter::toCvMat(SE3quat));
            }
            else
            {
                pKF->mTcwGBA.create(4, 4, CV_32F);
                Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
                pKF->mnBAGlobalForKF = nLoopKF;
            }
        }

        // Points
        for (size_t i = 0; i < vpMP.size(); i++)
        {
            if (vbNotIncludedMP[i])
                continue;

            MapPoint *pMP = vpMP[i];

            if (pMP->isBad())
                continue;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));

            if (nLoopKF == 0)
            {
                pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
                pMP->UpdateNormalAndDepth();
            }
            else
            {
                pMP->mPosGBA.create(3, 1, CV_32F);
                Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
                pMP->mnBAGlobalForKF = nLoopKF;
            }
        }
    }

    int Optimizer::PoseOptimization(Frame *pFrame)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        int nInitialCorrespondences = 0;

        // Set Frame vertex
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
        vSE3->setId(0);
        vSE3->setFixed(false);
        optimizer.addVertex(vSE3);

        // Set MapPoint vertices
        const int N = pFrame->N;

        vector<g2o::EdgeSE3ProjectXYZOnlyPose *> vpEdgesMono;
        vector<size_t> vnIndexEdgeMono;
        vpEdgesMono.reserve(N);
        vnIndexEdgeMono.reserve(N);

        vector<g2o::EdgeStereoSE3ProjectXYZOnlyPose *> vpEdgesStereo;
        vector<size_t> vnIndexEdgeStereo;
        vpEdgesStereo.reserve(N);
        vnIndexEdgeStereo.reserve(N);

        const float deltaMono = sqrt(5.991);
        const float deltaStereo = sqrt(7.815);

        {
            unique_lock<mutex> lock(MapPoint::mGlobalMutex);

            for (int i = 0; i < N; i++)
            {
                MapPoint *pMP = pFrame->mvpMapPoints[i];
                if (pMP)
                {
                    // Monocular observation
                    if (pFrame->mvuRight[i] < 0)
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        Eigen::Matrix<double, 2, 1> obs;
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZOnlyPose *e = new g2o::EdgeSE3ProjectXYZOnlyPose();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaMono);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        optimizer.addEdge(e);

                        vpEdgesMono.push_back(e);
                        vnIndexEdgeMono.push_back(i);
                    }
                    else // Stereo observation
                    {
                        nInitialCorrespondences++;
                        pFrame->mvbOutlier[i] = false;

                        // SET EDGE
                        Eigen::Matrix<double, 3, 1> obs;
                        const cv::KeyPoint &kpUn = pFrame->mvKeysUn[i];
                        const float &kp_ur = pFrame->mvuRight[i];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = new g2o::EdgeStereoSE3ProjectXYZOnlyPose();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
                        e->setMeasurement(obs);
                        const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(deltaStereo);

                        e->fx = pFrame->fx;
                        e->fy = pFrame->fy;
                        e->cx = pFrame->cx;
                        e->cy = pFrame->cy;
                        e->bf = pFrame->mbf;
                        cv::Mat Xw = pMP->GetWorldPos();
                        e->Xw[0] = Xw.at<float>(0);
                        e->Xw[1] = Xw.at<float>(1);
                        e->Xw[2] = Xw.at<float>(2);

                        optimizer.addEdge(e);

                        vpEdgesStereo.push_back(e);
                        vnIndexEdgeStereo.push_back(i);
                    }
                }
            }
        }

        if (nInitialCorrespondences < 3)
            return 0;

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        const float chi2Mono[4] = {5.991, 5.991, 5.991, 5.991};
        const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
        const int its[4] = {10, 10, 10, 10};

        int nBad = 0;
        for (size_t it = 0; it < 4; it++)
        {

            vSE3->setEstimate(Converter::toSE3Quat(pFrame->mTcw));
            optimizer.initializeOptimization(0);
            optimizer.optimize(its[it]);

            nBad = 0;
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZOnlyPose *e = vpEdgesMono[i];

                const size_t idx = vnIndexEdgeMono[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Mono[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    pFrame->mvbOutlier[idx] = false;
                    e->setLevel(0);
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZOnlyPose *e = vpEdgesStereo[i];

                const size_t idx = vnIndexEdgeStereo[i];

                if (pFrame->mvbOutlier[idx])
                {
                    e->computeError();
                }

                const float chi2 = e->chi2();

                if (chi2 > chi2Stereo[it])
                {
                    pFrame->mvbOutlier[idx] = true;
                    e->setLevel(1);
                    nBad++;
                }
                else
                {
                    e->setLevel(0);
                    pFrame->mvbOutlier[idx] = false;
                }

                if (it == 2)
                    e->setRobustKernel(0);
            }

            if (optimizer.edges().size() < 10)
                break;
        }

        // Recover optimized pose and return number of inliers
        g2o::VertexSE3Expmap *vSE3_recov = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(0));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
        cv::Mat pose = Converter::toCvMat(SE3quat_recov);
        pFrame->SetPose(pose);

        return nInitialCorrespondences - nBad;
    }

    void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap)
    {
        // Local KeyFrames: First Breath Search from Current Keyframe
        list<KeyFrame *> lLocalKeyFrames;

        lLocalKeyFrames.push_back(pKF);
        pKF->mnBALocalForKF = pKF->mnId;

        const vector<KeyFrame *> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
        for (int i = 0, iend = vNeighKFs.size(); i < iend; i++)
        {
            KeyFrame *pKFi = vNeighKFs[i];
            pKFi->mnBALocalForKF = pKF->mnId;
            if (!pKFi->isBad())
                lLocalKeyFrames.push_back(pKFi);
        }

        // Local MapPoints seen in Local KeyFrames
        list<MapPoint *> lLocalMapPoints;
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            vector<MapPoint *> vpMPs = (*lit)->GetMapPointMatches();
            for (vector<MapPoint *>::iterator vit = vpMPs.begin(), vend = vpMPs.end(); vit != vend; vit++)
            {
                MapPoint *pMP = *vit;
                if (pMP)
                    if (!pMP->isBad())
                        if (pMP->mnBALocalForKF != pKF->mnId)
                        {
                            lLocalMapPoints.push_back(pMP);
                            pMP->mnBALocalForKF = pKF->mnId;
                        }
            }
        }

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        list<KeyFrame *> lFixedCameras;
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            map<KeyFrame *, size_t> observations = (*lit)->GetObservations();
            for (map<KeyFrame *, size_t>::iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (pKFi->mnBALocalForKF != pKF->mnId && pKFi->mnBAFixedForKF != pKF->mnId)
                {
                    pKFi->mnBAFixedForKF = pKF->mnId;
                    if (!pKFi->isBad())
                        lFixedCameras.push_back(pKFi);
                }
            }
        }

        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolver_6_3::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

        g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        if (pbStopFlag)
            optimizer.setForceStopFlag(pbStopFlag);

        unsigned long maxKFid = 0;

        // Set Local KeyFrame vertices
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(pKFi->mnId == 0);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // Set Fixed KeyFrame vertices
        for (list<KeyFrame *>::iterator lit = lFixedCameras.begin(), lend = lFixedCameras.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;
            g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
            vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
            vSE3->setId(pKFi->mnId);
            vSE3->setFixed(true);
            optimizer.addVertex(vSE3);
            if (pKFi->mnId > maxKFid)
                maxKFid = pKFi->mnId;
        }

        // Set MapPoint vertices
        const int nExpectedSize = (lLocalKeyFrames.size() + lFixedCameras.size()) * lLocalMapPoints.size();

        vector<g2o::EdgeSE3ProjectXYZ *> vpEdgesMono;
        vpEdgesMono.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFMono;
        vpEdgeKFMono.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeMono;
        vpMapPointEdgeMono.reserve(nExpectedSize);

        vector<g2o::EdgeStereoSE3ProjectXYZ *> vpEdgesStereo;
        vpEdgesStereo.reserve(nExpectedSize);

        vector<KeyFrame *> vpEdgeKFStereo;
        vpEdgeKFStereo.reserve(nExpectedSize);

        vector<MapPoint *> vpMapPointEdgeStereo;
        vpMapPointEdgeStereo.reserve(nExpectedSize);

        const float thHuberMono = sqrt(5.991);
        const float thHuberStereo = sqrt(7.815);

        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
            vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
            int id = pMP->mnId + maxKFid + 1;
            vPoint->setId(id);
            vPoint->setMarginalized(true);
            optimizer.addVertex(vPoint);

            const map<KeyFrame *, size_t> observations = pMP->GetObservations();

            // Set edges
            for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(), mend = observations.end(); mit != mend; mit++)
            {
                KeyFrame *pKFi = mit->first;

                if (!pKFi->isBad())
                {
                    const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                    // Monocular observation
                    if (pKFi->mvuRight[mit->second] < 0)
                    {
                        Eigen::Matrix<double, 2, 1> obs;
                        obs << kpUn.pt.x, kpUn.pt.y;

                        g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberMono);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;

                        optimizer.addEdge(e);
                        vpEdgesMono.push_back(e);
                        vpEdgeKFMono.push_back(pKFi);
                        vpMapPointEdgeMono.push_back(pMP);
                    }
                    else // Stereo observation
                    {
                        Eigen::Matrix<double, 3, 1> obs;
                        const float kp_ur = pKFi->mvuRight[mit->second];
                        obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFi->mnId)));
                        e->setMeasurement(obs);
                        const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                        Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                        e->setInformation(Info);

                        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                        rk->setDelta(thHuberStereo);

                        e->fx = pKFi->fx;
                        e->fy = pKFi->fy;
                        e->cx = pKFi->cx;
                        e->cy = pKFi->cy;
                        e->bf = pKFi->mbf;

                        optimizer.addEdge(e);
                        vpEdgesStereo.push_back(e);
                        vpEdgeKFStereo.push_back(pKFi);
                        vpMapPointEdgeStereo.push_back(pMP);
                    }
                }
            }
        }

        if (pbStopFlag)
            if (*pbStopFlag)
                return;

        optimizer.initializeOptimization();
        optimizer.optimize(5);

        bool bDoMore = true;

        if (pbStopFlag)
            if (*pbStopFlag)
                bDoMore = false;

        if (bDoMore)
        {

            // Check inlier observations
            for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
            {
                g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
                MapPoint *pMP = vpMapPointEdgeMono[i];

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 5.991 || !e->isDepthPositive())
                {
                    e->setLevel(1);
                }

                e->setRobustKernel(0);
            }

            for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
            {
                g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
                MapPoint *pMP = vpMapPointEdgeStereo[i];

                if (pMP->isBad())
                    continue;

                if (e->chi2() > 7.815 || !e->isDepthPositive())
                {
                    e->setLevel(1);
                }

                e->setRobustKernel(0);
            }

            // Optimize again without the outliers

            optimizer.initializeOptimization(0);
            optimizer.optimize(10);
        }

        vector<pair<KeyFrame *, MapPoint *>> vToErase;
        vToErase.reserve(vpEdgesMono.size() + vpEdgesStereo.size());

        // Check inlier observations
        for (size_t i = 0, iend = vpEdgesMono.size(); i < iend; i++)
        {
            g2o::EdgeSE3ProjectXYZ *e = vpEdgesMono[i];
            MapPoint *pMP = vpMapPointEdgeMono[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 5.991 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFMono[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        for (size_t i = 0, iend = vpEdgesStereo.size(); i < iend; i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ *e = vpEdgesStereo[i];
            MapPoint *pMP = vpMapPointEdgeStereo[i];

            if (pMP->isBad())
                continue;

            if (e->chi2() > 7.815 || !e->isDepthPositive())
            {
                KeyFrame *pKFi = vpEdgeKFStereo[i];
                vToErase.push_back(make_pair(pKFi, pMP));
            }
        }

        // Get Map Mutex
        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        if (!vToErase.empty())
        {
            for (size_t i = 0; i < vToErase.size(); i++)
            {
                KeyFrame *pKFi = vToErase[i].first;
                MapPoint *pMPi = vToErase[i].second;
                pKFi->EraseMapPointMatch(pMPi);
                pMPi->EraseObservation(pKFi);
            }
        }

        // Recover optimized data

        // Keyframes
        for (list<KeyFrame *>::iterator lit = lLocalKeyFrames.begin(), lend = lLocalKeyFrames.end(); lit != lend; lit++)
        {
            KeyFrame *pKF = *lit;
            g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
            g2o::SE3Quat SE3quat = vSE3->estimate();
            pKF->SetPose(Converter::toCvMat(SE3quat));
        }

        // Points
        for (list<MapPoint *>::iterator lit = lLocalMapPoints.begin(), lend = lLocalMapPoints.end(); lit != lend; lit++)
        {
            MapPoint *pMP = *lit;
            g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(pMP->mnId + maxKFid + 1));
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        }
    }

    void Optimizer::OptimizeEssentialGraph(Map *pMap, KeyFrame *pLoopKF, KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *>> &LoopConnections, const bool &bFixScale)
    {
        // Setup optimizer
        g2o::SparseOptimizer optimizer;
        optimizer.setVerbose(false);
        g2o::BlockSolver_7_3::LinearSolverType *linearSolver =
            new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
        g2o::BlockSolver_7_3 *solver_ptr = new g2o::BlockSolver_7_3(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

        solver->setUserLambdaInit(1e-16);
        optimizer.setAlgorithm(solver);

        const vector<KeyFrame *> vpKFs = pMap->GetAllKeyFrames();
        const vector<MapPoint *> vpMPs = pMap->GetAllMapPoints();

        const unsigned int nMaxKFid = pMap->GetMaxKFid();

        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vScw(nMaxKFid + 1);
        vector<g2o::Sim3, Eigen::aligned_allocator<g2o::Sim3>> vCorrectedSwc(nMaxKFid + 1);
        vector<g2o::VertexSim3Expmap *> vpVertices(nMaxKFid + 1);

        const int minFeat = 100;

        // Set KeyFrame vertices
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];
            if (pKF->isBad())
                continue;
            g2o::VertexSim3Expmap *VSim3 = new g2o::VertexSim3Expmap();

            const int nIDi = pKF->mnId;

            LoopClosing::KeyFrameAndPose::const_iterator it = CorrectedSim3.find(pKF);

            if (it != CorrectedSim3.end())
            {
                vScw[nIDi] = it->second;
                VSim3->setEstimate(it->second);
            }
            else
            {
                Eigen::Matrix<double, 3, 3> Rcw = Converter::toMatrix3d(pKF->GetRotation());
                Eigen::Matrix<double, 3, 1> tcw = Converter::toVector3d(pKF->GetTranslation());
                g2o::Sim3 Siw(Rcw, tcw, 1.0);
                vScw[nIDi] = Siw;
                VSim3->setEstimate(Siw);
            }

            if (pKF == pLoopKF)
                VSim3->setFixed(true);

            VSim3->setId(nIDi);
            VSim3->setMarginalized(false);
            VSim3->_fix_scale = bFixScale;

            optimizer.addVertex(VSim3);

            vpVertices[nIDi] = VSim3;
        }

        set<pair<long unsigned int, long unsigned int>> sInsertedEdges;

        const Eigen::Matrix<double, 7, 7> matLambda = Eigen::Matrix<double, 7, 7>::Identity();

        // Set Loop edges
        for (map<KeyFrame *, set<KeyFrame *>>::const_iterator mit = LoopConnections.begin(), mend = LoopConnections.end(); mit != mend; mit++)
        {
            KeyFrame *pKF = mit->first;
            const long unsigned int nIDi = pKF->mnId;
            const set<KeyFrame *> &spConnections = mit->second;
            const g2o::Sim3 Siw = vScw[nIDi];
            const g2o::Sim3 Swi = Siw.inverse();

            for (set<KeyFrame *>::const_iterator sit = spConnections.begin(), send = spConnections.end(); sit != send; sit++)
            {
                const long unsigned int nIDj = (*sit)->mnId;
                if ((nIDi != pCurKF->mnId || nIDj != pLoopKF->mnId) && pKF->GetWeight(*sit) < minFeat)
                    continue;

                const g2o::Sim3 Sjw = vScw[nIDj];
                const g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;

                optimizer.addEdge(e);

                sInsertedEdges.insert(make_pair(min(nIDi, nIDj), max(nIDi, nIDj)));
            }
        }

        // Set normal edges
        for (size_t i = 0, iend = vpKFs.size(); i < iend; i++)
        {
            KeyFrame *pKF = vpKFs[i];

            const int nIDi = pKF->mnId;

            g2o::Sim3 Swi;

            LoopClosing::KeyFrameAndPose::const_iterator iti = NonCorrectedSim3.find(pKF);

            if (iti != NonCorrectedSim3.end())
                Swi = (iti->second).inverse();
            else
                Swi = vScw[nIDi].inverse();

            KeyFrame *pParentKF = pKF->GetParent();

            // Spanning tree edge
            if (pParentKF)
            {
                int nIDj = pParentKF->mnId;

                g2o::Sim3 Sjw;

                LoopClosing::KeyFrameAndPose::const_iterator itj = NonCorrectedSim3.find(pParentKF);

                if (itj != NonCorrectedSim3.end())
                    Sjw = itj->second;
                else
                    Sjw = vScw[nIDj];

                g2o::Sim3 Sji = Sjw * Swi;

                g2o::EdgeSim3 *e = new g2o::EdgeSim3();
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDj)));
                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                e->setMeasurement(Sji);

                e->information() = matLambda;
                optimizer.addEdge(e);
            }

            // Loop edges
            const set<KeyFrame *> sLoopEdges = pKF->GetLoopEdges();
            for (set<KeyFrame *>::const_iterator sit = sLoopEdges.begin(), send = sLoopEdges.end(); sit != send; sit++)
            {
                KeyFrame *pLKF = *sit;
                if (pLKF->mnId < pKF->mnId)
                {
                    g2o::Sim3 Slw;

                    LoopClosing::KeyFrameAndPose::const_iterator itl = NonCorrectedSim3.find(pLKF);

                    if (itl != NonCorrectedSim3.end())
                        Slw = itl->second;
                    else
                        Slw = vScw[pLKF->mnId];

                    g2o::Sim3 Sli = Slw * Swi;
                    g2o::EdgeSim3 *el = new g2o::EdgeSim3();
                    el->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pLKF->mnId)));
                    el->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                    el->setMeasurement(Sli);
                    el->information() = matLambda;
                    optimizer.addEdge(el);
                }
            }

            // Covisibility graph edges
            const vector<KeyFrame *> vpConnectedKFs = pKF->GetCovisiblesByWeight(minFeat);
            for (vector<KeyFrame *>::const_iterator vit = vpConnectedKFs.begin(); vit != vpConnectedKFs.end(); vit++)
            {
                KeyFrame *pKFn = *vit;
                if (pKFn && pKFn != pParentKF && !pKF->hasChild(pKFn) && !sLoopEdges.count(pKFn))
                {
                    if (!pKFn->isBad() && pKFn->mnId < pKF->mnId)
                    {
                        if (sInsertedEdges.count(make_pair(min(pKF->mnId, pKFn->mnId), max(pKF->mnId, pKFn->mnId))))
                            continue;

                        g2o::Sim3 Snw;

                        LoopClosing::KeyFrameAndPose::const_iterator itn = NonCorrectedSim3.find(pKFn);

                        if (itn != NonCorrectedSim3.end())
                            Snw = itn->second;
                        else
                            Snw = vScw[pKFn->mnId];

                        g2o::Sim3 Sni = Snw * Swi;

                        g2o::EdgeSim3 *en = new g2o::EdgeSim3();
                        en->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKFn->mnId)));
                        en->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(nIDi)));
                        en->setMeasurement(Sni);
                        en->information() = matLambda;
                        optimizer.addEdge(en);
                    }
                }
            }
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(20);

        unique_lock<mutex> lock(pMap->mMutexMapUpdate);

        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        for (size_t i = 0; i < vpKFs.size(); i++)
        {
            KeyFrame *pKFi = vpKFs[i];

            const int nIDi = pKFi->mnId;

            g2o::VertexSim3Expmap *VSim3 = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(nIDi));
            g2o::Sim3 CorrectedSiw = VSim3->estimate();
            vCorrectedSwc[nIDi] = CorrectedSiw.inverse();
            Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = CorrectedSiw.translation();
            double s = CorrectedSiw.scale();

            eigt *= (1. / s); //[R t/s;0 1]

            cv::Mat Tiw = Converter::toCvSE3(eigR, eigt);

            pKFi->SetPose(Tiw);
        }

        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        for (size_t i = 0, iend = vpMPs.size(); i < iend; i++)
        {
            MapPoint *pMP = vpMPs[i];

            if (pMP->isBad())
                continue;

            int nIDr;
            if (pMP->mnCorrectedByKF == pCurKF->mnId)
            {
                nIDr = pMP->mnCorrectedReference;
            }
            else
            {
                KeyFrame *pRefKF = pMP->GetReferenceKeyFrame();
                nIDr = pRefKF->mnId;
            }

            g2o::Sim3 Srw = vScw[nIDr];
            g2o::Sim3 correctedSwr = vCorrectedSwc[nIDr];

            cv::Mat P3Dw = pMP->GetWorldPos();
            Eigen::Matrix<double, 3, 1> eigP3Dw = Converter::toVector3d(P3Dw);
            Eigen::Matrix<double, 3, 1> eigCorrectedP3Dw = correctedSwr.map(Srw.map(eigP3Dw));

            cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
            pMP->SetWorldPos(cvCorrectedP3Dw);

            pMP->UpdateNormalAndDepth();
        }
    }

    int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
    {
        g2o::SparseOptimizer optimizer;
        g2o::BlockSolverX::LinearSolverType *linearSolver;

        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();

        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);

        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        optimizer.setAlgorithm(solver);

        // Calibration
        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;

        // Camera poses
        const cv::Mat R1w = pKF1->GetRotation();
        const cv::Mat t1w = pKF1->GetTranslation();
        const cv::Mat R2w = pKF2->GetRotation();
        const cv::Mat t2w = pKF2->GetTranslation();

        // Set Sim3 vertex
        g2o::VertexSim3Expmap *vSim3 = new g2o::VertexSim3Expmap();
        vSim3->_fix_scale = bFixScale;
        vSim3->setEstimate(g2oS12);
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->_principle_point1[0] = K1.at<float>(0, 2);
        vSim3->_principle_point1[1] = K1.at<float>(1, 2);
        vSim3->_focal_length1[0] = K1.at<float>(0, 0);
        vSim3->_focal_length1[1] = K1.at<float>(1, 1);
        vSim3->_principle_point2[0] = K2.at<float>(0, 2);
        vSim3->_principle_point2[1] = K2.at<float>(1, 2);
        vSim3->_focal_length2[0] = K2.at<float>(0, 0);
        vSim3->_focal_length2[1] = K2.at<float>(1, 1);
        optimizer.addVertex(vSim3);

        // Set MapPoint vertices
        const int N = vpMatches1.size();
        const vector<MapPoint *> vpMapPoints1 = pKF1->GetMapPointMatches();
        vector<g2o::EdgeSim3ProjectXYZ *> vpEdges12;
        vector<g2o::EdgeInverseSim3ProjectXYZ *> vpEdges21;
        vector<size_t> vnIndexEdge;

        vnIndexEdge.reserve(2 * N);
        vpEdges12.reserve(2 * N);
        vpEdges21.reserve(2 * N);

        const float deltaHuber = sqrt(th2);

        int nCorrespondences = 0;

        for (int i = 0; i < N; i++)
        {
            if (!vpMatches1[i])
                continue;

            MapPoint *pMP1 = vpMapPoints1[i];
            MapPoint *pMP2 = vpMatches1[i];

            const int id1 = 2 * i + 1;
            const int id2 = 2 * (i + 1);

            const int i2 = pMP2->GetIndexInKeyFrame(pKF2);

            if (pMP1 && pMP2)
            {
                if (!pMP1->isBad() && !pMP2->isBad() && i2 >= 0)
                {
                    g2o::VertexSBAPointXYZ *vPoint1 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D1w = pMP1->GetWorldPos();
                    cv::Mat P3D1c = R1w * P3D1w + t1w;
                    vPoint1->setEstimate(Converter::toVector3d(P3D1c));
                    vPoint1->setId(id1);
                    vPoint1->setFixed(true);
                    optimizer.addVertex(vPoint1);

                    g2o::VertexSBAPointXYZ *vPoint2 = new g2o::VertexSBAPointXYZ();
                    cv::Mat P3D2w = pMP2->GetWorldPos();
                    cv::Mat P3D2c = R2w * P3D2w + t2w;
                    vPoint2->setEstimate(Converter::toVector3d(P3D2c));
                    vPoint2->setId(id2);
                    vPoint2->setFixed(true);
                    optimizer.addVertex(vPoint2);
                }
                else
                    continue;
            }
            else
                continue;

            nCorrespondences++;

            // Set edge x1 = S12*X2
            Eigen::Matrix<double, 2, 1> obs1;
            const cv::KeyPoint &kpUn1 = pKF1->mvKeysUn[i];
            obs1 << kpUn1.pt.x, kpUn1.pt.y;

            g2o::EdgeSim3ProjectXYZ *e12 = new g2o::EdgeSim3ProjectXYZ();
            e12->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id2)));
            e12->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e12->setMeasurement(obs1);
            const float &invSigmaSquare1 = pKF1->mvInvLevelSigma2[kpUn1.octave];
            e12->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare1);

            g2o::RobustKernelHuber *rk1 = new g2o::RobustKernelHuber;
            e12->setRobustKernel(rk1);
            rk1->setDelta(deltaHuber);
            optimizer.addEdge(e12);

            // Set edge x2 = S21*X1
            Eigen::Matrix<double, 2, 1> obs2;
            const cv::KeyPoint &kpUn2 = pKF2->mvKeysUn[i2];
            obs2 << kpUn2.pt.x, kpUn2.pt.y;

            g2o::EdgeInverseSim3ProjectXYZ *e21 = new g2o::EdgeInverseSim3ProjectXYZ();

            e21->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id1)));
            e21->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(0)));
            e21->setMeasurement(obs2);
            float invSigmaSquare2 = pKF2->mvInvLevelSigma2[kpUn2.octave];
            e21->setInformation(Eigen::Matrix2d::Identity() * invSigmaSquare2);

            g2o::RobustKernelHuber *rk2 = new g2o::RobustKernelHuber;
            e21->setRobustKernel(rk2);
            rk2->setDelta(deltaHuber);
            optimizer.addEdge(e21);

            vpEdges12.push_back(e12);
            vpEdges21.push_back(e21);
            vnIndexEdge.push_back(i);
        }

        // Optimize!
        optimizer.initializeOptimization();
        optimizer.optimize(5);

        // Check inliers
        int nBad = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
                optimizer.removeEdge(e12);
                optimizer.removeEdge(e21);
                vpEdges12[i] = static_cast<g2o::EdgeSim3ProjectXYZ *>(NULL);
                vpEdges21[i] = static_cast<g2o::EdgeInverseSim3ProjectXYZ *>(NULL);
                nBad++;
            }
        }

        int nMoreIterations;
        if (nBad > 0)
            nMoreIterations = 10;
        else
            nMoreIterations = 5;

        if (nCorrespondences - nBad < 10)
            return 0;

        // Optimize again only with inliers

        optimizer.initializeOptimization();
        optimizer.optimize(nMoreIterations);

        int nIn = 0;
        for (size_t i = 0; i < vpEdges12.size(); i++)
        {
            g2o::EdgeSim3ProjectXYZ *e12 = vpEdges12[i];
            g2o::EdgeInverseSim3ProjectXYZ *e21 = vpEdges21[i];
            if (!e12 || !e21)
                continue;

            if (e12->chi2() > th2 || e21->chi2() > th2)
            {
                size_t idx = vnIndexEdge[i];
                vpMatches1[idx] = static_cast<MapPoint *>(NULL);
            }
            else
                nIn++;
        }

        // Recover optimized Sim3
        g2o::VertexSim3Expmap *vSim3_recov = static_cast<g2o::VertexSim3Expmap *>(optimizer.vertex(0));
        g2oS12 = vSim3_recov->estimate();

        return nIn;
    }

} // namespace ORB_SLAM
