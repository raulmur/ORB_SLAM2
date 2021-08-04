/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

/******************************************************************************
* Author:   Steffen Urban                                              *
* Contact:  urbste@gmail.com                                          *
* License:  Copyright (c) 2016 Steffen Urban, ANU. All rights reserved.      *
*                                                                            *
* Redistribution and use in source and binary forms, with or without         *
* modification, are permitted provided that the following conditions         *
* are met:                                                                   *
* * Redistributions of source code must retain the above copyright           *
*   notice, this list of conditions and the following disclaimer.            *
* * Redistributions in binary form must reproduce the above copyright        *
*   notice, this list of conditions and the following disclaimer in the      *
*   documentation and/or other materials provided with the distribution.     *
* * Neither the name of ANU nor the names of its contributors may be         *
*   used to endorse or promote products derived from this software without   *
*   specific prior written permission.                                       *
*                                                                            *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"*
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE  *
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE *
* ARE DISCLAIMED. IN NO EVENT SHALL ANU OR THE CONTRIBUTORS BE LIABLE        *
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL *
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR *
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER *
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT         *
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY  *
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF     *
* SUCH DAMAGE.                                                               *
******************************************************************************/

#ifndef ORB_SLAM3_MLPNPSOLVER_H
#define ORB_SLAM3_MLPNPSOLVER_H

#include "MapPoint.h"
#include "Frame.h"

#include<Eigen/Dense>
#include<Eigen/Sparse>

namespace ORB_SLAM3{
    class MLPnPsolver {
    public:
        MLPnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

        ~MLPnPsolver();

        void SetRansacParameters(double probability = 0.99, int minInliers = 8, int maxIterations = 300, int minSet = 6, float epsilon = 0.4,
                                 float th2 = 5.991);

        cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

        //Type definitions needed by the original code

        /** A 3-vector of unit length used to describe landmark observations/bearings
         *  in camera frames (always expressed in camera frames)
         */
        typedef Eigen::Vector3d bearingVector_t;

        /** An array of bearing-vectors */
        typedef std::vector<bearingVector_t, Eigen::aligned_allocator<bearingVector_t> >
                bearingVectors_t;

        /** A 2-matrix containing the 2D covariance information of a bearing vector
        */
        typedef Eigen::Matrix2d cov2_mat_t;

        /** A 3-matrix containing the 3D covariance information of a bearing vector */
        typedef Eigen::Matrix3d cov3_mat_t;

        /** An array of 3D covariance matrices */
        typedef std::vector<cov3_mat_t, Eigen::aligned_allocator<cov3_mat_t> >
                cov3_mats_t;

        /** A 3-vector describing a point in 3D-space */
        typedef Eigen::Vector3d point_t;

        /** An array of 3D-points */
        typedef std::vector<point_t, Eigen::aligned_allocator<point_t> >
                points_t;

        /** A homogeneous 3-vector describing a point in 3D-space */
        typedef Eigen::Vector4d point4_t;

        /** An array of homogeneous 3D-points */
        typedef std::vector<point4_t, Eigen::aligned_allocator<point4_t> >
                points4_t;

        /** A 3-vector containing the rodrigues parameters of a rotation matrix */
        typedef Eigen::Vector3d rodrigues_t;

        /** A rotation matrix */
        typedef Eigen::Matrix3d rotation_t;

        /** A 3x4 transformation matrix containing rotation \f$ \mathbf{R} \f$ and
         *  translation \f$ \mathbf{t} \f$ as follows:
         *  \f$ \left( \begin{array}{cc} \mathbf{R} & \mathbf{t} \end{array} \right) \f$
         */
        typedef Eigen::Matrix<double,3,4> transformation_t;

        /** A 3-vector describing a translation/camera position */
        typedef Eigen::Vector3d translation_t;



    private:
        void CheckInliers();
        bool Refine();

        //Functions from de original MLPnP code

        /*
         * Computes the camera pose given 3D points coordinates (in the camera reference
         * system), the camera rays and (optionally) the covariance matrix of those camera rays.
         * Result is stored in solution
         */
        void computePose(
                const bearingVectors_t & f,
                const points_t & p,
                const cov3_mats_t & covMats,
                const std::vector<int>& indices,
                transformation_t & result);

        void mlpnp_gn(Eigen::VectorXd& x,
                      const points_t& pts,
                      const std::vector<Eigen::MatrixXd>& nullspaces,
                      const Eigen::SparseMatrix<double> Kll,
                      bool use_cov);

        void mlpnp_residuals_and_jacs(
                const Eigen::VectorXd& x,
                const points_t& pts,
                const std::vector<Eigen::MatrixXd>& nullspaces,
                Eigen::VectorXd& r,
                Eigen::MatrixXd& fjac,
                bool getJacs);

        void mlpnpJacs(
            const point_t& pt,
            const Eigen::Vector3d& nullspace_r,
            const Eigen::Vector3d& nullspace_s,
            const rodrigues_t& w,
            const translation_t& t,
            Eigen::MatrixXd& jacs);

        //Auxiliar methods

        /**
        * \brief Compute a rotation matrix from Rodrigues axis angle.
        *
        * \param[in] omega The Rodrigues-parameters of a rotation.
        * \return The 3x3 rotation matrix.
        */
        Eigen::Matrix3d rodrigues2rot(const Eigen::Vector3d & omega);

        /**
        * \brief Compute the Rodrigues-parameters of a rotation matrix.
        *
        * \param[in] R The 3x3 rotation matrix.
        * \return The Rodrigues-parameters.
        */
        Eigen::Vector3d rot2rodrigues(const Eigen::Matrix3d & R);

        //----------------------------------------------------
        //Fields of the solver
        //----------------------------------------------------
        vector<MapPoint*> mvpMapPointMatches;

        // 2D Points
        vector<cv::Point2f> mvP2D;
        //Substitued by bearing vectors
        bearingVectors_t mvBearingVecs;

        vector<float> mvSigma2;

        // 3D Points
        //vector<cv::Point3f> mvP3Dw;
        points_t mvP3Dw;

        // Index in Frame
        vector<size_t> mvKeyPointIndices;

        // Current Estimation
        double mRi[3][3];
        double mti[3];
        cv::Mat mTcwi;
        vector<bool> mvbInliersi;
        int mnInliersi;

        // Current Ransac State
        int mnIterations;
        vector<bool> mvbBestInliers;
        int mnBestInliers;
        cv::Mat mBestTcw;

        // Refined
        cv::Mat mRefinedTcw;
        vector<bool> mvbRefinedInliers;
        int mnRefinedInliers;

        // Number of Correspondences
        int N;

        // Indices for random selection [0 .. N-1]
        vector<size_t> mvAllIndices;

        // RANSAC probability
        double mRansacProb;

        // RANSAC min inliers
        int mRansacMinInliers;

        // RANSAC max iterations
        int mRansacMaxIts;

        // RANSAC expected inliers/total ratio
        float mRansacEpsilon;

        // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
        float mRansacTh;

        // RANSAC Minimun Set used at each iteration
        int mRansacMinSet;

        // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
        vector<float> mvMaxError;

        GeometricCamera* mpCamera;

    };

}




#endif //ORB_SLAM3_MLPNPSOLVER_H
