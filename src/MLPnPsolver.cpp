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

#include "MLPnPsolver.h"

#include <Eigen/Sparse>


namespace ORB_SLAM3 {
    MLPnPsolver::MLPnPsolver(const Frame &F, const vector<MapPoint *> &vpMapPointMatches):
            mnInliersi(0), mnIterations(0), mnBestInliers(0), N(0), mpCamera(F.mpCamera){
        mvpMapPointMatches = vpMapPointMatches;
        mvBearingVecs.reserve(F.mvpMapPoints.size());
        mvP2D.reserve(F.mvpMapPoints.size());
        mvSigma2.reserve(F.mvpMapPoints.size());
        mvP3Dw.reserve(F.mvpMapPoints.size());
        mvKeyPointIndices.reserve(F.mvpMapPoints.size());
        mvAllIndices.reserve(F.mvpMapPoints.size());

        int idx = 0;
        for(size_t i = 0, iend = mvpMapPointMatches.size(); i < iend; i++){
            MapPoint* pMP = vpMapPointMatches[i];

            if(pMP){
                if(!pMP -> isBad()){
                    if(i >= F.mvKeysUn.size()) continue;
                    const cv::KeyPoint &kp = F.mvKeysUn[i];

                    mvP2D.push_back(kp.pt);
                    mvSigma2.push_back(F.mvLevelSigma2[kp.octave]);

                    //Bearing vector should be normalized
                    cv::Point3f cv_br = mpCamera->unproject(kp.pt);
                    cv_br /= cv_br.z;
                    bearingVector_t br(cv_br.x,cv_br.y,cv_br.z);
                    mvBearingVecs.push_back(br);

                    //3D coordinates
                    cv::Mat cv_pos = pMP -> GetWorldPos();
                    point_t pos(cv_pos.at<float>(0),cv_pos.at<float>(1),cv_pos.at<float>(2));
                    mvP3Dw.push_back(pos);

                    mvKeyPointIndices.push_back(i);
                    mvAllIndices.push_back(idx);

                    idx++;
                }
            }
        }

        SetRansacParameters();
    }

    //RANSAC methods
	cv::Mat MLPnPsolver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers){
		bNoMore = false;
	    vbInliers.clear();
	    nInliers=0;

	    if(N<mRansacMinInliers)
	    {
	        bNoMore = true;
	        return cv::Mat();
	    }

	    vector<size_t> vAvailableIndices;

	    int nCurrentIterations = 0;
	    while(mnIterations<mRansacMaxIts || nCurrentIterations<nIterations)
	    {
	        nCurrentIterations++;
	        mnIterations++;

	        vAvailableIndices = mvAllIndices;

            //Bearing vectors and 3D points used for this ransac iteration
            bearingVectors_t bearingVecs(mRansacMinSet);
            points_t p3DS(mRansacMinSet);
            vector<int> indexes(mRansacMinSet);

	        // Get min set of points
	        for(short i = 0; i < mRansacMinSet; ++i)
	        {
	            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

	            int idx = vAvailableIndices[randi];

                bearingVecs[i] = mvBearingVecs[idx];
                p3DS[i] = mvP3Dw[idx];
                indexes[i] = i;

	            vAvailableIndices[randi] = vAvailableIndices.back();
	            vAvailableIndices.pop_back();
	        }

            //By the moment, we are using MLPnP without covariance info
            cov3_mats_t covs(1);

            //Result
            transformation_t result;

	        // Compute camera pose
            computePose(bearingVecs,p3DS,covs,indexes,result);

            //Save result
            mRi[0][0] = result(0,0);
            mRi[0][1] = result(0,1);
            mRi[0][2] = result(0,2);

            mRi[1][0] = result(1,0);
            mRi[1][1] = result(1,1);
            mRi[1][2] = result(1,2);

            mRi[2][0] = result(2,0);
            mRi[2][1] = result(2,1);
            mRi[2][2] = result(2,2);

            mti[0] = result(0,3);mti[1] = result(1,3);mti[2] = result(2,3);

	        // Check inliers
	        CheckInliers();

	        if(mnInliersi>=mRansacMinInliers)
	        {
	            // If it is the best solution so far, save it
	            if(mnInliersi>mnBestInliers)
	            {
	                mvbBestInliers = mvbInliersi;
	                mnBestInliers = mnInliersi;

	                cv::Mat Rcw(3,3,CV_64F,mRi);
	                cv::Mat tcw(3,1,CV_64F,mti);
	                Rcw.convertTo(Rcw,CV_32F);
	                tcw.convertTo(tcw,CV_32F);
	                mBestTcw = cv::Mat::eye(4,4,CV_32F);
	                Rcw.copyTo(mBestTcw.rowRange(0,3).colRange(0,3));
	                tcw.copyTo(mBestTcw.rowRange(0,3).col(3));
	            }

	            if(Refine())
	            {
	                nInliers = mnRefinedInliers;
	                vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
	                for(int i=0; i<N; i++)
	                {
	                    if(mvbRefinedInliers[i])
	                        vbInliers[mvKeyPointIndices[i]] = true;
	                }
	                return mRefinedTcw.clone();
	            }

	        }
	    }

	    if(mnIterations>=mRansacMaxIts)
	    {
	        bNoMore=true;
	        if(mnBestInliers>=mRansacMinInliers)
	        {
	            nInliers=mnBestInliers;
	            vbInliers = vector<bool>(mvpMapPointMatches.size(),false);
	            for(int i=0; i<N; i++)
	            {
	                if(mvbBestInliers[i])
	                    vbInliers[mvKeyPointIndices[i]] = true;
	            }
	            return mBestTcw.clone();
	        }
	    }

	    return cv::Mat();
	}

	void MLPnPsolver::SetRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2){
		mRansacProb = probability;
	    mRansacMinInliers = minInliers;
	    mRansacMaxIts = maxIterations;
	    mRansacEpsilon = epsilon;
	    mRansacMinSet = minSet;

	    N = mvP2D.size(); // number of correspondences

	    mvbInliersi.resize(N);

	    // Adjust Parameters according to number of correspondences
	    int nMinInliers = N*mRansacEpsilon;
	    if(nMinInliers<mRansacMinInliers)
	        nMinInliers=mRansacMinInliers;
	    if(nMinInliers<minSet)
	        nMinInliers=minSet;
	    mRansacMinInliers = nMinInliers;

	    if(mRansacEpsilon<(float)mRansacMinInliers/N)
	        mRansacEpsilon=(float)mRansacMinInliers/N;

	    // Set RANSAC iterations according to probability, epsilon, and max iterations
	    int nIterations;

	    if(mRansacMinInliers==N)
	        nIterations=1;
	    else
	        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

	    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

	    mvMaxError.resize(mvSigma2.size());
	    for(size_t i=0; i<mvSigma2.size(); i++)
	        mvMaxError[i] = mvSigma2[i]*th2;
	}

    void MLPnPsolver::CheckInliers(){
        mnInliersi=0;

        for(int i=0; i<N; i++)
        {
            point_t p = mvP3Dw[i];
            cv::Point3f P3Dw(p(0),p(1),p(2));
            cv::Point2f P2D = mvP2D[i];

            float xc = mRi[0][0]*P3Dw.x+mRi[0][1]*P3Dw.y+mRi[0][2]*P3Dw.z+mti[0];
            float yc = mRi[1][0]*P3Dw.x+mRi[1][1]*P3Dw.y+mRi[1][2]*P3Dw.z+mti[1];
            float zc = mRi[2][0]*P3Dw.x+mRi[2][1]*P3Dw.y+mRi[2][2]*P3Dw.z+mti[2];

            cv::Point3f P3Dc(xc,yc,zc);
            cv::Point2f uv = mpCamera->project(P3Dc);

            float distX = P2D.x-uv.x;
            float distY = P2D.y-uv.y;

            float error2 = distX*distX+distY*distY;

            if(error2<mvMaxError[i])
            {
                mvbInliersi[i]=true;
                mnInliersi++;
            }
            else
            {
                mvbInliersi[i]=false;
            }
        }
    }

    bool MLPnPsolver::Refine(){
        vector<int> vIndices;
        vIndices.reserve(mvbBestInliers.size());

        for(size_t i=0; i<mvbBestInliers.size(); i++)
        {
            if(mvbBestInliers[i])
            {
                vIndices.push_back(i);
            }
        }

        //Bearing vectors and 3D points used for this ransac iteration
        bearingVectors_t bearingVecs;
        points_t p3DS;
        vector<int> indexes;

        for(size_t i=0; i<vIndices.size(); i++)
        {
            int idx = vIndices[i];

            bearingVecs.push_back(mvBearingVecs[idx]);
            p3DS.push_back(mvP3Dw[idx]);
            indexes.push_back(i);
        }

        //By the moment, we are using MLPnP without covariance info
        cov3_mats_t covs(1);

        //Result
        transformation_t result;

        // Compute camera pose
        computePose(bearingVecs,p3DS,covs,indexes,result);

        // Check inliers
        CheckInliers();

        mnRefinedInliers =mnInliersi;
        mvbRefinedInliers = mvbInliersi;

        if(mnInliersi>mRansacMinInliers)
        {
            cv::Mat Rcw(3,3,CV_64F,mRi);
            cv::Mat tcw(3,1,CV_64F,mti);
            Rcw.convertTo(Rcw,CV_32F);
            tcw.convertTo(tcw,CV_32F);
            mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
            Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));
            return true;
        }

        return false;
    }

	//MLPnP methods
    void MLPnPsolver::computePose(const bearingVectors_t &f, const points_t &p, const cov3_mats_t &covMats,
                                  const std::vector<int> &indices, transformation_t &result) {
        size_t numberCorrespondences = indices.size();
        assert(numberCorrespondences > 5);

        bool planar = false;
        // compute the nullspace of all vectors
        std::vector<Eigen::MatrixXd> nullspaces(numberCorrespondences);
        Eigen::MatrixXd points3(3, numberCorrespondences);
        points_t points3v(numberCorrespondences);
        points4_t points4v(numberCorrespondences);
        for (size_t i = 0; i < numberCorrespondences; i++) {
            bearingVector_t f_current = f[indices[i]];
            points3.col(i) = p[indices[i]];
            // nullspace of right vector
            Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::HouseholderQRPreconditioner>
                    svd_f(f_current.transpose(), Eigen::ComputeFullV);
            nullspaces[i] = svd_f.matrixV().block(0, 1, 3, 2);
            points3v[i] = p[indices[i]];
        }

        //////////////////////////////////////
        // 1. test if we have a planar scene
        //////////////////////////////////////

        Eigen::Matrix3d planarTest = points3 * points3.transpose();
        Eigen::FullPivHouseholderQR<Eigen::Matrix3d> rankTest(planarTest);
        Eigen::Matrix3d eigenRot;
        eigenRot.setIdentity();

        // if yes -> transform points to new eigen frame
        //rankTest.setThreshold(1e-10);
        if (rankTest.rank() == 2) {
            planar = true;
            // self adjoint is faster and more accurate than general eigen solvers
            // also has closed form solution for 3x3 self-adjoint matrices
            // in addition this solver sorts the eigenvalues in increasing order
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(planarTest);
            eigenRot = eigen_solver.eigenvectors().real();
            eigenRot.transposeInPlace();
            for (size_t i = 0; i < numberCorrespondences; i++)
                points3.col(i) = eigenRot * points3.col(i);
        }
        //////////////////////////////////////
        // 2. stochastic model
        //////////////////////////////////////
        Eigen::SparseMatrix<double> P(2 * numberCorrespondences,
                                      2 * numberCorrespondences);
        bool use_cov = false;
        P.setIdentity(); // standard

        // if we do have covariance information
        // -> fill covariance matrix
        if (covMats.size() == numberCorrespondences) {
            use_cov = true;
            int l = 0;
            for (size_t i = 0; i < numberCorrespondences; ++i) {
                // invert matrix
                cov2_mat_t temp = nullspaces[i].transpose() * covMats[i] * nullspaces[i];
                temp = temp.inverse().eval();
                P.coeffRef(l, l) = temp(0, 0);
                P.coeffRef(l, l + 1) = temp(0, 1);
                P.coeffRef(l + 1, l) = temp(1, 0);
                P.coeffRef(l + 1, l + 1) = temp(1, 1);
                l += 2;
            }
        }

        //////////////////////////////////////
        // 3. fill the design matrix A
        //////////////////////////////////////
        const int rowsA = 2 * numberCorrespondences;
        int colsA = 12;
        Eigen::MatrixXd A;
        if (planar) {
            colsA = 9;
            A = Eigen::MatrixXd(rowsA, 9);
        } else
            A = Eigen::MatrixXd(rowsA, 12);
        A.setZero();

        // fill design matrix
        if (planar) {
            for (size_t i = 0; i < numberCorrespondences; ++i) {
                point_t pt3_current = points3.col(i);

                // r12
                A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[1];
                A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[1];
                // r13
                A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[2];
                A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[2];
                // r22
                A(2 * i, 2) = nullspaces[i](1, 0) * pt3_current[1];
                A(2 * i + 1, 2) = nullspaces[i](1, 1) * pt3_current[1];
                // r23
                A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[2];
                A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[2];
                // r32
                A(2 * i, 4) = nullspaces[i](2, 0) * pt3_current[1];
                A(2 * i + 1, 4) = nullspaces[i](2, 1) * pt3_current[1];
                // r33
                A(2 * i, 5) = nullspaces[i](2, 0) * pt3_current[2];
                A(2 * i + 1, 5) = nullspaces[i](2, 1) * pt3_current[2];
                // t1
                A(2 * i, 6) = nullspaces[i](0, 0);
                A(2 * i + 1, 6) = nullspaces[i](0, 1);
                // t2
                A(2 * i, 7) = nullspaces[i](1, 0);
                A(2 * i + 1, 7) = nullspaces[i](1, 1);
                // t3
                A(2 * i, 8) = nullspaces[i](2, 0);
                A(2 * i + 1, 8) = nullspaces[i](2, 1);
            }
        } else {
            for (size_t i = 0; i < numberCorrespondences; ++i) {
                point_t pt3_current = points3.col(i);

                // r11
                A(2 * i, 0) = nullspaces[i](0, 0) * pt3_current[0];
                A(2 * i + 1, 0) = nullspaces[i](0, 1) * pt3_current[0];
                // r12
                A(2 * i, 1) = nullspaces[i](0, 0) * pt3_current[1];
                A(2 * i + 1, 1) = nullspaces[i](0, 1) * pt3_current[1];
                // r13
                A(2 * i, 2) = nullspaces[i](0, 0) * pt3_current[2];
                A(2 * i + 1, 2) = nullspaces[i](0, 1) * pt3_current[2];
                // r21
                A(2 * i, 3) = nullspaces[i](1, 0) * pt3_current[0];
                A(2 * i + 1, 3) = nullspaces[i](1, 1) * pt3_current[0];
                // r22
                A(2 * i, 4) = nullspaces[i](1, 0) * pt3_current[1];
                A(2 * i + 1, 4) = nullspaces[i](1, 1) * pt3_current[1];
                // r23
                A(2 * i, 5) = nullspaces[i](1, 0) * pt3_current[2];
                A(2 * i + 1, 5) = nullspaces[i](1, 1) * pt3_current[2];
                // r31
                A(2 * i, 6) = nullspaces[i](2, 0) * pt3_current[0];
                A(2 * i + 1, 6) = nullspaces[i](2, 1) * pt3_current[0];
                // r32
                A(2 * i, 7) = nullspaces[i](2, 0) * pt3_current[1];
                A(2 * i + 1, 7) = nullspaces[i](2, 1) * pt3_current[1];
                // r33
                A(2 * i, 8) = nullspaces[i](2, 0) * pt3_current[2];
                A(2 * i + 1, 8) = nullspaces[i](2, 1) * pt3_current[2];
                // t1
                A(2 * i, 9) = nullspaces[i](0, 0);
                A(2 * i + 1, 9) = nullspaces[i](0, 1);
                // t2
                A(2 * i, 10) = nullspaces[i](1, 0);
                A(2 * i + 1, 10) = nullspaces[i](1, 1);
                // t3
                A(2 * i, 11) = nullspaces[i](2, 0);
                A(2 * i + 1, 11) = nullspaces[i](2, 1);
            }
        }

        //////////////////////////////////////
        // 4. solve least squares
        //////////////////////////////////////
        Eigen::MatrixXd AtPA;
        if (use_cov)
            AtPA = A.transpose() * P * A; // setting up the full normal equations seems to be unstable
        else
            AtPA = A.transpose() * A;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd_A(AtPA, Eigen::ComputeFullV);
        Eigen::MatrixXd result1 = svd_A.matrixV().col(colsA - 1);

        ////////////////////////////////
        // now we treat the results differently,
        // depending on the scene (planar or not)
        ////////////////////////////////
        //transformation_t T_final;
        rotation_t Rout;
        translation_t tout;
        if (planar) // planar case
        {
            rotation_t tmp;
            // until now, we only estimated
            // row one and two of the transposed rotation matrix
            tmp << 0.0, result1(0, 0), result1(1, 0),
                    0.0, result1(2, 0), result1(3, 0),
                    0.0, result1(4, 0), result1(5, 0);
            // row 3
            tmp.col(0) = tmp.col(1).cross(tmp.col(2));
            tmp.transposeInPlace();

            double scale = 1.0 / std::sqrt(std::abs(tmp.col(1).norm() * tmp.col(2).norm()));
            // find best rotation matrix in frobenius sense
            Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
            rotation_t Rout1 = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();
            // test if we found a good rotation matrix
            if (Rout1.determinant() < 0)
                Rout1 *= -1.0;
            // rotate this matrix back using the eigen frame
            Rout1 = eigenRot.transpose() * Rout1;

            translation_t t = scale * translation_t(result1(6, 0), result1(7, 0), result1(8, 0));
            Rout1.transposeInPlace();
            Rout1 *= -1;
            if (Rout1.determinant() < 0.0)
                Rout1.col(2) *= -1;
            // now we have to find the best out of 4 combinations
            rotation_t R1, R2;
            R1.col(0) = Rout1.col(0);
            R1.col(1) = Rout1.col(1);
            R1.col(2) = Rout1.col(2);
            R2.col(0) = -Rout1.col(0);
            R2.col(1) = -Rout1.col(1);
            R2.col(2) = Rout1.col(2);

            vector<transformation_t, Eigen::aligned_allocator<transformation_t>> Ts(4);
            Ts[0].block<3, 3>(0, 0) = R1;
            Ts[0].block<3, 1>(0, 3) = t;
            Ts[1].block<3, 3>(0, 0) = R1;
            Ts[1].block<3, 1>(0, 3) = -t;
            Ts[2].block<3, 3>(0, 0) = R2;
            Ts[2].block<3, 1>(0, 3) = t;
            Ts[3].block<3, 3>(0, 0) = R2;
            Ts[3].block<3, 1>(0, 3) = -t;

            vector<double> normVal(4);
            for (int i = 0; i < 4; ++i) {
                point_t reproPt;
                double norms = 0.0;
                for (int p = 0; p < 6; ++p) {
                    reproPt = Ts[i].block<3, 3>(0, 0) * points3v[p] + Ts[i].block<3, 1>(0, 3);
                    reproPt = reproPt / reproPt.norm();
                    norms += (1.0 - reproPt.transpose() * f[indices[p]]);
                }
                normVal[i] = norms;
            }
            std::vector<double>::iterator
                    findMinRepro = std::min_element(std::begin(normVal), std::end(normVal));
            int idx = std::distance(std::begin(normVal), findMinRepro);
            Rout = Ts[idx].block<3, 3>(0, 0);
            tout = Ts[idx].block<3, 1>(0, 3);
        } else // non-planar
        {
            rotation_t tmp;
            tmp << result1(0, 0), result1(3, 0), result1(6, 0),
                    result1(1, 0), result1(4, 0), result1(7, 0),
                    result1(2, 0), result1(5, 0), result1(8, 0);
            // get the scale
            double scale = 1.0 /
                           std::pow(std::abs(tmp.col(0).norm() * tmp.col(1).norm() * tmp.col(2).norm()), 1.0 / 3.0);
            //double scale = 1.0 / std::sqrt(std::abs(tmp.col(0).norm() * tmp.col(1).norm()));
            // find best rotation matrix in frobenius sense
            Eigen::JacobiSVD<Eigen::MatrixXd> svd_R_frob(tmp, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Rout = svd_R_frob.matrixU() * svd_R_frob.matrixV().transpose();
            // test if we found a good rotation matrix
            if (Rout.determinant() < 0)
                Rout *= -1.0;
            // scale translation
            tout = Rout * (scale * translation_t(result1(9, 0), result1(10, 0), result1(11, 0)));

            // find correct direction in terms of reprojection error, just take the first 6 correspondences
            vector<double> error(2);
            vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Ts(2);
            for (int s = 0; s < 2; ++s) {
                error[s] = 0.0;
                Ts[s] = Eigen::Matrix4d::Identity();
                Ts[s].block<3, 3>(0, 0) = Rout;
                if (s == 0)
                    Ts[s].block<3, 1>(0, 3) = tout;
                else
                    Ts[s].block<3, 1>(0, 3) = -tout;
                Ts[s] = Ts[s].inverse().eval();
                for (int p = 0; p < 6; ++p) {
                    bearingVector_t v = Ts[s].block<3, 3>(0, 0) * points3v[p] + Ts[s].block<3, 1>(0, 3);
                    v = v / v.norm();
                    error[s] += (1.0 - v.transpose() * f[indices[p]]);
                }
            }
            if (error[0] < error[1])
                tout = Ts[0].block<3, 1>(0, 3);
            else
                tout = Ts[1].block<3, 1>(0, 3);
            Rout = Ts[0].block<3, 3>(0, 0);

        }

        //////////////////////////////////////
        // 5. gauss newton
        //////////////////////////////////////
        rodrigues_t omega = rot2rodrigues(Rout);
        Eigen::VectorXd minx(6);
        minx[0] = omega[0];
        minx[1] = omega[1];
        minx[2] = omega[2];
        minx[3] = tout[0];
        minx[4] = tout[1];
        minx[5] = tout[2];

        mlpnp_gn(minx, points3v, nullspaces, P, use_cov);

        Rout = rodrigues2rot(rodrigues_t(minx[0], minx[1], minx[2]));
        tout = translation_t(minx[3], minx[4], minx[5]);
        // result inverse as opengv uses this convention
        result.block<3, 3>(0, 0) = Rout;//Rout.transpose();
        result.block<3, 1>(0, 3) = tout;//-result.block<3, 3>(0, 0) * tout;
    }

    Eigen::Matrix3d MLPnPsolver::rodrigues2rot(const Eigen::Vector3d &omega) {
        rotation_t R = Eigen::Matrix3d::Identity();

        Eigen::Matrix3d skewW;
        skewW << 0.0, -omega(2), omega(1),
                omega(2), 0.0, -omega(0),
                -omega(1), omega(0), 0.0;

        double omega_norm = omega.norm();

        if (omega_norm > std::numeric_limits<double>::epsilon())
            R = R + sin(omega_norm) / omega_norm * skewW
                + (1 - cos(omega_norm)) / (omega_norm * omega_norm) * (skewW * skewW);

        return R;
    }

    Eigen::Vector3d MLPnPsolver::rot2rodrigues(const Eigen::Matrix3d &R) {
        rodrigues_t omega;
        omega << 0.0, 0.0, 0.0;

        double trace = R.trace() - 1.0;
        double wnorm = acos(trace / 2.0);
        if (wnorm > std::numeric_limits<double>::epsilon())
        {
            omega[0] = (R(2, 1) - R(1, 2));
            omega[1] = (R(0, 2) - R(2, 0));
            omega[2] = (R(1, 0) - R(0, 1));
            double sc = wnorm / (2.0*sin(wnorm));
            omega *= sc;
        }
        return omega;
    }

    void MLPnPsolver::mlpnp_gn(Eigen::VectorXd &x, const points_t &pts, const std::vector<Eigen::MatrixXd> &nullspaces,
                               const Eigen::SparseMatrix<double> Kll, bool use_cov) {
        const int numObservations = pts.size();
        const int numUnknowns = 6;
        // check redundancy
        assert((2 * numObservations - numUnknowns) > 0);

        // =============
        // set all matrices up
        // =============

        Eigen::VectorXd r(2 * numObservations);
        Eigen::VectorXd rd(2 * numObservations);
        Eigen::MatrixXd Jac(2 * numObservations, numUnknowns);
        Eigen::VectorXd g(numUnknowns, 1);
        Eigen::VectorXd dx(numUnknowns, 1); // result vector

        Jac.setZero();
        r.setZero();
        dx.setZero();
        g.setZero();

        int it_cnt = 0;
        bool stop = false;
        const int maxIt = 5;
        double epsP = 1e-5;

        Eigen::MatrixXd JacTSKll;
        Eigen::MatrixXd A;
        // solve simple gradient descent
        while (it_cnt < maxIt && !stop) {
            mlpnp_residuals_and_jacs(x, pts,
                                     nullspaces,
                                     r, Jac, true);

            if (use_cov)
                JacTSKll = Jac.transpose() * Kll;
            else
                JacTSKll = Jac.transpose();

            A = JacTSKll * Jac;

            // get system matrix
            g = JacTSKll * r;

            // solve
            Eigen::LDLT<Eigen::MatrixXd> chol(A);
            dx = chol.solve(g);
            // this is to prevent the solution from falling into a wrong minimum
            // if the linear estimate is spurious
            if (dx.array().abs().maxCoeff() > 5.0 || dx.array().abs().minCoeff() > 1.0)
                break;
            // observation update
            Eigen::MatrixXd dl = Jac * dx;
            if (dl.array().abs().maxCoeff() < epsP) {
                stop = true;
                x = x - dx;
                break;
            } else
                x = x - dx;

            ++it_cnt;
        }//while
        // result
    }

    void MLPnPsolver::mlpnp_residuals_and_jacs(const Eigen::VectorXd &x, const points_t &pts,
                                               const std::vector<Eigen::MatrixXd> &nullspaces, Eigen::VectorXd &r,
                                               Eigen::MatrixXd &fjac, bool getJacs) {
        rodrigues_t w(x[0], x[1], x[2]);
        translation_t T(x[3], x[4], x[5]);

        rotation_t R = rodrigues2rot(w);
        int ii = 0;

        Eigen::MatrixXd jacs(2, 6);

        for (int i = 0; i < pts.size(); ++i)
        {
            Eigen::Vector3d ptCam = R*pts[i] + T;
            ptCam /= ptCam.norm();

            r[ii] = nullspaces[i].col(0).transpose()*ptCam;
            r[ii + 1] = nullspaces[i].col(1).transpose()*ptCam;
            if (getJacs)
            {
                // jacs
                mlpnpJacs(pts[i],
                          nullspaces[i].col(0), nullspaces[i].col(1),
                          w, T,
                          jacs);

                // r
                fjac(ii, 0) = jacs(0, 0);
                fjac(ii, 1) = jacs(0, 1);
                fjac(ii, 2) = jacs(0, 2);

                fjac(ii, 3) = jacs(0, 3);
                fjac(ii, 4) = jacs(0, 4);
                fjac(ii, 5) = jacs(0, 5);
                // s
                fjac(ii + 1, 0) = jacs(1, 0);
                fjac(ii + 1, 1) = jacs(1, 1);
                fjac(ii + 1, 2) = jacs(1, 2);

                fjac(ii + 1, 3) = jacs(1, 3);
                fjac(ii + 1, 4) = jacs(1, 4);
                fjac(ii + 1, 5) = jacs(1, 5);

            }
            ii += 2;
        }
    }

    void MLPnPsolver::mlpnpJacs(const point_t& pt, const Eigen::Vector3d& nullspace_r,
            					const Eigen::Vector3d& nullspace_s, const rodrigues_t& w,
            					const translation_t& t, Eigen::MatrixXd& jacs){
    	double r1 = nullspace_r[0];
		double r2 = nullspace_r[1];
		double r3 = nullspace_r[2];

		double s1 = nullspace_s[0];
		double s2 = nullspace_s[1];
		double s3 = nullspace_s[2];

		double X1 = pt[0];
		double Y1 = pt[1];
		double Z1 = pt[2];

		double w1 = w[0];
		double w2 = w[1];
		double w3 = w[2];

		double t1 = t[0];
		double t2 = t[1];
		double t3 = t[2];

		 double t5 = w1*w1;
		 double t6 = w2*w2;
		 double t7 = w3*w3;
		 double t8 = t5+t6+t7;
		 double t9 = sqrt(t8);
		 double t10 = sin(t9);
		 double t11 = 1.0/sqrt(t8);
		 double t12 = cos(t9);
		 double  t13 = t12-1.0;
		 double  t14 = 1.0/t8;
		 double  t16 = t10*t11*w3;
		 double t17 = t13*t14*w1*w2;
		 double t19 = t10*t11*w2;
		 double t20 = t13*t14*w1*w3;
		 double t24 = t6+t7;
		 double t27 = t16+t17;
		 double t28 = Y1*t27;
		 double t29 = t19-t20;
		 double t30 = Z1*t29;
		 double t31 = t13*t14*t24;
		 double t32 = t31+1.0;
		 double t33 = X1*t32;
		 double t15 = t1-t28+t30+t33;
		 double t21 = t10*t11*w1;
		 double t22 = t13*t14*w2*w3;
		 double t45 = t5+t7;
		 double t53 = t16-t17;
		 double t54 = X1*t53;
		 double t55 = t21+t22;
		 double t56 = Z1*t55;
		 double t57 = t13*t14*t45;
		 double t58 = t57+1.0;
		 double t59 = Y1*t58;
		 double t18 = t2+t54-t56+t59;
		 double t34 = t5+t6;
		 double t38 = t19+t20;
		 double t39 = X1*t38;
		 double t40 = t21-t22;
		 double t41 = Y1*t40;
		 double t42 = t13*t14*t34;
		 double t43 = t42+1.0;
		 double t44 = Z1*t43;
		 double t23 = t3-t39+t41+t44;
		 double t25 = 1.0/pow(t8,3.0/2.0);
		 double t26 = 1.0/(t8*t8);
		 double t35 = t12*t14*w1*w2;
		 double t36 = t5*t10*t25*w3;
		 double t37 = t5*t13*t26*w3*2.0;
		 double t46 = t10*t25*w1*w3;
		 double t47 = t5*t10*t25*w2;
		 double t48 = t5*t13*t26*w2*2.0;
		 double t49 = t10*t11;
		 double t50 = t5*t12*t14;
		 double t51 = t13*t26*w1*w2*w3*2.0;
		 double t52 = t10*t25*w1*w2*w3;
		 double t60 = t15*t15;
		 double t61 = t18*t18;
		 double t62 = t23*t23;
		 double t63 = t60+t61+t62;
		 double t64 = t5*t10*t25;
		 double t65 = 1.0/sqrt(t63);
		 double t66 = Y1*r2*t6;
		 double t67 = Z1*r3*t7;
		 double t68 = r1*t1*t5;
		 double t69 = r1*t1*t6;
		 double t70 = r1*t1*t7;
		 double  t71 = r2*t2*t5;
		 double  t72 = r2*t2*t6;
		 double  t73 = r2*t2*t7;
		 double  t74 = r3*t3*t5;
		 double  t75 = r3*t3*t6;
		 double  t76 = r3*t3*t7;
		 double  t77 = X1*r1*t5;
		 double  t78 = X1*r2*w1*w2;
		 double  t79 = X1*r3*w1*w3;
		 double  t80 = Y1*r1*w1*w2;
		 double  t81 = Y1*r3*w2*w3;
		 double  t82 = Z1*r1*w1*w3;
		 double  t83 = Z1*r2*w2*w3;
		 double  t84 = X1*r1*t6*t12;
		 double  t85 = X1*r1*t7*t12;
		 double  t86 = Y1*r2*t5*t12;
		 double  t87 = Y1*r2*t7*t12;
		 double  t88 = Z1*r3*t5*t12;
		 double  t89 = Z1*r3*t6*t12;
		 double  t90 = X1*r2*t9*t10*w3;
		 double  t91 = Y1*r3*t9*t10*w1;
		 double  t92 = Z1*r1*t9*t10*w2;
		 double  t102 = X1*r3*t9*t10*w2;
		 double  t103 = Y1*r1*t9*t10*w3;
		 double  t104 = Z1*r2*t9*t10*w1;
		 double  t105 = X1*r2*t12*w1*w2;
		 double  t106 = X1*r3*t12*w1*w3;
		 double  t107 = Y1*r1*t12*w1*w2;
		 double  t108 = Y1*r3*t12*w2*w3;
		 double  t109 = Z1*r1*t12*w1*w3;
		 double  t110 = Z1*r2*t12*w2*w3;
		 double  t93 = t66+t67+t68+t69+t70+t71+t72+t73+t74+t75+t76+t77+t78+t79+t80+t81+t82+t83+t84+t85+t86+t87+t88+t89+t90+t91+t92-t102-t103-t104-t105-t106-t107-t108-t109-t110;
		 double  t94 = t10*t25*w1*w2;
		 double  t95 = t6*t10*t25*w3;
		 double  t96 = t6*t13*t26*w3*2.0;
		 double  t97 = t12*t14*w2*w3;
		 double  t98 = t6*t10*t25*w1;
		 double  t99 = t6*t13*t26*w1*2.0;
		 double  t100 = t6*t10*t25;
		 double  t101 = 1.0/pow(t63,3.0/2.0);
		 double  t111 = t6*t12*t14;
		 double  t112 = t10*t25*w2*w3;
		 double  t113 = t12*t14*w1*w3;
		 double  t114 = t7*t10*t25*w2;
		 double  t115 = t7*t13*t26*w2*2.0;
		 double  t116 = t7*t10*t25*w1;
		 double  t117 = t7*t13*t26*w1*2.0;
		 double  t118 = t7*t12*t14;
		 double  t119 = t13*t24*t26*w1*2.0;
		 double  t120 = t10*t24*t25*w1;
		 double  t121 = t119+t120;
		 double  t122 = t13*t26*t34*w1*2.0;
		 double  t123 = t10*t25*t34*w1;
		 double  t131 = t13*t14*w1*2.0;
		 double  t124 = t122+t123-t131;
		 double  t139 = t13*t14*w3;
		 double  t125 = -t35+t36+t37+t94-t139;
		 double  t126 = X1*t125;
		 double  t127 = t49+t50+t51+t52-t64;
		 double  t128 = Y1*t127;
		 double  t129 = t126+t128-Z1*t124;
		 double  t130 = t23*t129*2.0;
		 double  t132 = t13*t26*t45*w1*2.0;
		 double  t133 = t10*t25*t45*w1;
		 double  t138 = t13*t14*w2;
		 double  t134 = -t46+t47+t48+t113-t138;
		 double  t135 = X1*t134;
		 double  t136 = -t49-t50+t51+t52+t64;
		 double  t137 = Z1*t136;
		 double  t140 = X1*s1*t5;
		 double  t141 = Y1*s2*t6;
		 double  t142 = Z1*s3*t7;
		 double  t143 = s1*t1*t5;
		 double  t144 = s1*t1*t6;
		 double  t145 = s1*t1*t7;
		 double  t146 = s2*t2*t5;
		 double  t147 = s2*t2*t6;
		 double  t148 = s2*t2*t7;
		 double  t149 = s3*t3*t5;
		 double  t150 = s3*t3*t6;
		 double  t151 = s3*t3*t7;
		 double  t152 = X1*s2*w1*w2;
		 double  t153 = X1*s3*w1*w3;
		 double  t154 = Y1*s1*w1*w2;
		 double  t155 = Y1*s3*w2*w3;
		 double  t156 = Z1*s1*w1*w3;
		 double  t157 = Z1*s2*w2*w3;
		 double  t158 = X1*s1*t6*t12;
		 double  t159 = X1*s1*t7*t12;
		 double  t160 = Y1*s2*t5*t12;
		 double  t161 = Y1*s2*t7*t12;
		 double  t162 = Z1*s3*t5*t12;
		 double  t163 = Z1*s3*t6*t12;
		 double  t164 = X1*s2*t9*t10*w3;
		 double  t165 = Y1*s3*t9*t10*w1;
		 double  t166 = Z1*s1*t9*t10*w2;
		 double  t183 = X1*s3*t9*t10*w2;
		 double  t184 = Y1*s1*t9*t10*w3;
		 double  t185 = Z1*s2*t9*t10*w1;
		 double  t186 = X1*s2*t12*w1*w2;
		 double  t187 = X1*s3*t12*w1*w3;
		 double  t188 = Y1*s1*t12*w1*w2;
		 double  t189 = Y1*s3*t12*w2*w3;
		 double  t190 = Z1*s1*t12*w1*w3;
		 double  t191 = Z1*s2*t12*w2*w3;
		 double  t167 = t140+t141+t142+t143+t144+t145+t146+t147+t148+t149+t150+t151+t152+t153+t154+t155+t156+t157+t158+t159+t160+t161+t162+t163+t164+t165+t166-t183-t184-t185-t186-t187-t188-t189-t190-t191;
		 double  t168 = t13*t26*t45*w2*2.0;
		 double  t169 = t10*t25*t45*w2;
		 double  t170 = t168+t169;
		 double  t171 = t13*t26*t34*w2*2.0;
		 double  t172 = t10*t25*t34*w2;
		 double  t176 = t13*t14*w2*2.0;
		 double  t173 = t171+t172-t176;
		 double  t174 = -t49+t51+t52+t100-t111;
		 double  t175 = X1*t174;
		 double  t177 = t13*t24*t26*w2*2.0;
		 double  t178 = t10*t24*t25*w2;
		 double  t192 = t13*t14*w1;
		 double  t179 = -t97+t98+t99+t112-t192;
		 double  t180 = Y1*t179;
		 double  t181 = t49+t51+t52-t100+t111;
		 double  t182 = Z1*t181;
		 double  t193 = t13*t26*t34*w3*2.0;
		 double  t194 = t10*t25*t34*w3;
		 double  t195 = t193+t194;
		 double  t196 = t13*t26*t45*w3*2.0;
		 double  t197 = t10*t25*t45*w3;
		 double  t200 = t13*t14*w3*2.0;
		 double  t198 = t196+t197-t200;
		 double  t199 = t7*t10*t25;
		 double  t201 = t13*t24*t26*w3*2.0;
		 double  t202 = t10*t24*t25*w3;
		 double  t203 = -t49+t51+t52-t118+t199;
		 double  t204 = Y1*t203;
		 double  t205 = t1*2.0;
		 double  t206 = Z1*t29*2.0;
		 double  t207 = X1*t32*2.0;
		 double  t208 = t205+t206+t207-Y1*t27*2.0;
		 double  t209 = t2*2.0;
		 double  t210 = X1*t53*2.0;
		 double  t211 = Y1*t58*2.0;
		 double  t212 = t209+t210+t211-Z1*t55*2.0;
		 double  t213 = t3*2.0;
		 double  t214 = Y1*t40*2.0;
		 double  t215 = Z1*t43*2.0;
		 double  t216 = t213+t214+t215-X1*t38*2.0;
		jacs(0, 0) = t14*t65*(X1*r1*w1*2.0+X1*r2*w2+X1*r3*w3+Y1*r1*w2+Z1*r1*w3+r1*t1*w1*2.0+r2*t2*w1*2.0+r3*t3*w1*2.0+Y1*r3*t5*t12+Y1*r3*t9*t10-Z1*r2*t5*t12-Z1*r2*t9*t10-X1*r2*t12*w2-X1*r3*t12*w3-Y1*r1*t12*w2+Y1*r2*t12*w1*2.0-Z1*r1*t12*w3+Z1*r3*t12*w1*2.0+Y1*r3*t5*t10*t11-Z1*r2*t5*t10*t11+X1*r2*t12*w1*w3-X1*r3*t12*w1*w2-Y1*r1*t12*w1*w3+Z1*r1*t12*w1*w2-Y1*r1*t10*t11*w1*w3+Z1*r1*t10*t11*w1*w2-X1*r1*t6*t10*t11*w1-X1*r1*t7*t10*t11*w1+X1*r2*t5*t10*t11*w2+X1*r3*t5*t10*t11*w3+Y1*r1*t5*t10*t11*w2-Y1*r2*t5*t10*t11*w1-Y1*r2*t7*t10*t11*w1+Z1*r1*t5*t10*t11*w3-Z1*r3*t5*t10*t11*w1-Z1*r3*t6*t10*t11*w1+X1*r2*t10*t11*w1*w3-X1*r3*t10*t11*w1*w2+Y1*r3*t10*t11*w1*w2*w3+Z1*r2*t10*t11*w1*w2*w3)-t26*t65*t93*w1*2.0-t14*t93*t101*(t130+t15*(-X1*t121+Y1*(t46+t47+t48-t13*t14*w2-t12*t14*w1*w3)+Z1*(t35+t36+t37-t13*t14*w3-t10*t25*w1*w2))*2.0+t18*(t135+t137-Y1*(t132+t133-t13*t14*w1*2.0))*2.0)*(1.0/2.0);
		jacs(0, 1) = t14*t65*(X1*r2*w1+Y1*r1*w1+Y1*r2*w2*2.0+Y1*r3*w3+Z1*r2*w3+r1*t1*w2*2.0+r2*t2*w2*2.0+r3*t3*w2*2.0-X1*r3*t6*t12-X1*r3*t9*t10+Z1*r1*t6*t12+Z1*r1*t9*t10+X1*r1*t12*w2*2.0-X1*r2*t12*w1-Y1*r1*t12*w1-Y1*r3*t12*w3-Z1*r2*t12*w3+Z1*r3*t12*w2*2.0-X1*r3*t6*t10*t11+Z1*r1*t6*t10*t11+X1*r2*t12*w2*w3-Y1*r1*t12*w2*w3+Y1*r3*t12*w1*w2-Z1*r2*t12*w1*w2-Y1*r1*t10*t11*w2*w3+Y1*r3*t10*t11*w1*w2-Z1*r2*t10*t11*w1*w2-X1*r1*t6*t10*t11*w2+X1*r2*t6*t10*t11*w1-X1*r1*t7*t10*t11*w2+Y1*r1*t6*t10*t11*w1-Y1*r2*t5*t10*t11*w2-Y1*r2*t7*t10*t11*w2+Y1*r3*t6*t10*t11*w3-Z1*r3*t5*t10*t11*w2+Z1*r2*t6*t10*t11*w3-Z1*r3*t6*t10*t11*w2+X1*r2*t10*t11*w2*w3+X1*r3*t10*t11*w1*w2*w3+Z1*r1*t10*t11*w1*w2*w3)-t26*t65*t93*w2*2.0-t14*t93*t101*(t18*(Z1*(-t35+t94+t95+t96-t13*t14*w3)-Y1*t170+X1*(t97+t98+t99-t13*t14*w1-t10*t25*w2*w3))*2.0+t15*(t180+t182-X1*(t177+t178-t13*t14*w2*2.0))*2.0+t23*(t175+Y1*(t35-t94+t95+t96-t13*t14*w3)-Z1*t173)*2.0)*(1.0/2.0);
		jacs(0, 2) = t14*t65*(X1*r3*w1+Y1*r3*w2+Z1*r1*w1+Z1*r2*w2+Z1*r3*w3*2.0+r1*t1*w3*2.0+r2*t2*w3*2.0+r3*t3*w3*2.0+X1*r2*t7*t12+X1*r2*t9*t10-Y1*r1*t7*t12-Y1*r1*t9*t10+X1*r1*t12*w3*2.0-X1*r3*t12*w1+Y1*r2*t12*w3*2.0-Y1*r3*t12*w2-Z1*r1*t12*w1-Z1*r2*t12*w2+X1*r2*t7*t10*t11-Y1*r1*t7*t10*t11-X1*r3*t12*w2*w3+Y1*r3*t12*w1*w3+Z1*r1*t12*w2*w3-Z1*r2*t12*w1*w3+Y1*r3*t10*t11*w1*w3+Z1*r1*t10*t11*w2*w3-Z1*r2*t10*t11*w1*w3-X1*r1*t6*t10*t11*w3-X1*r1*t7*t10*t11*w3+X1*r3*t7*t10*t11*w1-Y1*r2*t5*t10*t11*w3-Y1*r2*t7*t10*t11*w3+Y1*r3*t7*t10*t11*w2+Z1*r1*t7*t10*t11*w1+Z1*r2*t7*t10*t11*w2-Z1*r3*t5*t10*t11*w3-Z1*r3*t6*t10*t11*w3-X1*r3*t10*t11*w2*w3+X1*r2*t10*t11*w1*w2*w3+Y1*r1*t10*t11*w1*w2*w3)-t26*t65*t93*w3*2.0-t14*t93*t101*(t18*(Z1*(t46-t113+t114+t115-t13*t14*w2)-Y1*t198+X1*(t49+t51+t52+t118-t7*t10*t25))*2.0+t23*(X1*(-t97+t112+t116+t117-t13*t14*w1)+Y1*(-t46+t113+t114+t115-t13*t14*w2)-Z1*t195)*2.0+t15*(t204+Z1*(t97-t112+t116+t117-t13*t14*w1)-X1*(t201+t202-t13*t14*w3*2.0))*2.0)*(1.0/2.0);
		jacs(0, 3) = r1*t65-t14*t93*t101*t208*(1.0/2.0);
		jacs(0, 4) = r2*t65-t14*t93*t101*t212*(1.0/2.0);
		jacs(0, 5) = r3*t65-t14*t93*t101*t216*(1.0/2.0);
		jacs(1, 0) = t14*t65*(X1*s1*w1*2.0+X1*s2*w2+X1*s3*w3+Y1*s1*w2+Z1*s1*w3+s1*t1*w1*2.0+s2*t2*w1*2.0+s3*t3*w1*2.0+Y1*s3*t5*t12+Y1*s3*t9*t10-Z1*s2*t5*t12-Z1*s2*t9*t10-X1*s2*t12*w2-X1*s3*t12*w3-Y1*s1*t12*w2+Y1*s2*t12*w1*2.0-Z1*s1*t12*w3+Z1*s3*t12*w1*2.0+Y1*s3*t5*t10*t11-Z1*s2*t5*t10*t11+X1*s2*t12*w1*w3-X1*s3*t12*w1*w2-Y1*s1*t12*w1*w3+Z1*s1*t12*w1*w2+X1*s2*t10*t11*w1*w3-X1*s3*t10*t11*w1*w2-Y1*s1*t10*t11*w1*w3+Z1*s1*t10*t11*w1*w2-X1*s1*t6*t10*t11*w1-X1*s1*t7*t10*t11*w1+X1*s2*t5*t10*t11*w2+X1*s3*t5*t10*t11*w3+Y1*s1*t5*t10*t11*w2-Y1*s2*t5*t10*t11*w1-Y1*s2*t7*t10*t11*w1+Z1*s1*t5*t10*t11*w3-Z1*s3*t5*t10*t11*w1-Z1*s3*t6*t10*t11*w1+Y1*s3*t10*t11*w1*w2*w3+Z1*s2*t10*t11*w1*w2*w3)-t14*t101*t167*(t130+t15*(Y1*(t46+t47+t48-t113-t138)+Z1*(t35+t36+t37-t94-t139)-X1*t121)*2.0+t18*(t135+t137-Y1*(-t131+t132+t133))*2.0)*(1.0/2.0)-t26*t65*t167*w1*2.0;
		jacs(1, 1) = t14*t65*(X1*s2*w1+Y1*s1*w1+Y1*s2*w2*2.0+Y1*s3*w3+Z1*s2*w3+s1*t1*w2*2.0+s2*t2*w2*2.0+s3*t3*w2*2.0-X1*s3*t6*t12-X1*s3*t9*t10+Z1*s1*t6*t12+Z1*s1*t9*t10+X1*s1*t12*w2*2.0-X1*s2*t12*w1-Y1*s1*t12*w1-Y1*s3*t12*w3-Z1*s2*t12*w3+Z1*s3*t12*w2*2.0-X1*s3*t6*t10*t11+Z1*s1*t6*t10*t11+X1*s2*t12*w2*w3-Y1*s1*t12*w2*w3+Y1*s3*t12*w1*w2-Z1*s2*t12*w1*w2+X1*s2*t10*t11*w2*w3-Y1*s1*t10*t11*w2*w3+Y1*s3*t10*t11*w1*w2-Z1*s2*t10*t11*w1*w2-X1*s1*t6*t10*t11*w2+X1*s2*t6*t10*t11*w1-X1*s1*t7*t10*t11*w2+Y1*s1*t6*t10*t11*w1-Y1*s2*t5*t10*t11*w2-Y1*s2*t7*t10*t11*w2+Y1*s3*t6*t10*t11*w3-Z1*s3*t5*t10*t11*w2+Z1*s2*t6*t10*t11*w3-Z1*s3*t6*t10*t11*w2+X1*s3*t10*t11*w1*w2*w3+Z1*s1*t10*t11*w1*w2*w3)-t26*t65*t167*w2*2.0-t14*t101*t167*(t18*(X1*(t97+t98+t99-t112-t192)+Z1*(-t35+t94+t95+t96-t139)-Y1*t170)*2.0+t15*(t180+t182-X1*(-t176+t177+t178))*2.0+t23*(t175+Y1*(t35-t94+t95+t96-t139)-Z1*t173)*2.0)*(1.0/2.0);
		jacs(1, 2) = t14*t65*(X1*s3*w1+Y1*s3*w2+Z1*s1*w1+Z1*s2*w2+Z1*s3*w3*2.0+s1*t1*w3*2.0+s2*t2*w3*2.0+s3*t3*w3*2.0+X1*s2*t7*t12+X1*s2*t9*t10-Y1*s1*t7*t12-Y1*s1*t9*t10+X1*s1*t12*w3*2.0-X1*s3*t12*w1+Y1*s2*t12*w3*2.0-Y1*s3*t12*w2-Z1*s1*t12*w1-Z1*s2*t12*w2+X1*s2*t7*t10*t11-Y1*s1*t7*t10*t11-X1*s3*t12*w2*w3+Y1*s3*t12*w1*w3+Z1*s1*t12*w2*w3-Z1*s2*t12*w1*w3-X1*s3*t10*t11*w2*w3+Y1*s3*t10*t11*w1*w3+Z1*s1*t10*t11*w2*w3-Z1*s2*t10*t11*w1*w3-X1*s1*t6*t10*t11*w3-X1*s1*t7*t10*t11*w3+X1*s3*t7*t10*t11*w1-Y1*s2*t5*t10*t11*w3-Y1*s2*t7*t10*t11*w3+Y1*s3*t7*t10*t11*w2+Z1*s1*t7*t10*t11*w1+Z1*s2*t7*t10*t11*w2-Z1*s3*t5*t10*t11*w3-Z1*s3*t6*t10*t11*w3+X1*s2*t10*t11*w1*w2*w3+Y1*s1*t10*t11*w1*w2*w3)-t26*t65*t167*w3*2.0-t14*t101*t167*(t18*(Z1*(t46-t113+t114+t115-t138)-Y1*t198+X1*(t49+t51+t52+t118-t199))*2.0+t23*(X1*(-t97+t112+t116+t117-t192)+Y1*(-t46+t113+t114+t115-t138)-Z1*t195)*2.0+t15*(t204+Z1*(t97-t112+t116+t117-t192)-X1*(-t200+t201+t202))*2.0)*(1.0/2.0);
		jacs(1, 3) = s1*t65-t14*t101*t167*t208*(1.0/2.0);
		jacs(1, 4) = s2*t65-t14*t101*t167*t212*(1.0/2.0);
		jacs(1, 5) = s3*t65-t14*t101*t167*t216*(1.0/2.0);
    }
}//End namespace ORB_SLAM2
